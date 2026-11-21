"""Bridge the browser worker to the same kinematics used by the desktop lab."""

import csv
import json
import math
import re
from dataclasses import asdict, dataclass
from io import StringIO
from pathlib import Path, PureWindowsPath
from tempfile import TemporaryDirectory

import numpy as np

from JacobianKinematicsLab.interference import INTERFERENCE_TOLERANCE_M, find_interferences
from JacobianKinematicsLab.model import (
    CORNERS,
    EXAMPLES,
    HardpointTable,
    Model,
    load_model,
    make_motion,
    read_hardpoint_table,
)
from JacobianKinematicsLab.project import (
    _project_column,
    _read_deformable_members,
    _read_settings,
    save_project,
)
from JacobianKinematicsLab.results import _write_csv, damper_histories, export_results, motion_ratio
from JacobianKinematicsLab.solver import solve
from JacobianKinematicsLab.suspension import (
    damper_axles,
    has_front_axle,
    has_steering,
    sweep_corners,
    sweep_dampers,
    sweep_front_axle,
    sweep_suspension,
)
from JacobianKinematicsLab.wheel_frame import TireSetup, WheelFrame


REFERENCE_CSV = EXAMPLES / "hardpoints.csv"
PROJECT_FIELDS = {"points", "pushrod_mounts", "tires", "motion", "deformable_members", "mechanism"}


@dataclass(frozen=True)
class Project:
    filename: str
    table: HardpointTable
    points: dict[str, tuple[float, float, float]]
    model: Model
    tires: dict[str, TireSetup]
    motion: dict
    mechanism: dict | None
    builtin: bool


_project = None
_result = None
_ratio = None


def _finite(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must be a finite number.") from error
    if isinstance(value, bool) or not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number.")
    return number


def _object(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object.")
    return value


def _json_object(value, label):
    if isinstance(value, str):
        value = json.loads(value.lstrip("\ufeff"))
    return _object(value, label)


def _hardpoints(value):
    items = _object(value, "Hardpoints")
    if not items:
        raise ValueError("At least one hardpoint is required.")
    points = {}
    for raw_name, xyz in items.items():
        name = raw_name.strip()
        if not name or name in points:
            raise ValueError(f"Hardpoint names must be nonblank and unique. Check {raw_name!r}.")
        if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
            raise ValueError(f"Hardpoint {name} requires three XYZ coordinates in metres.")
        points[name] = tuple(_finite(value, f"{name} {axis}") for axis, value in zip("XYZ", xyz))
    return points


def _table_from_points(points, file):
    with file.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("name", "x_m", "y_m", "z_m"))
        writer.writerows((name, *xyz) for name, xyz in points.items())
    return read_hardpoint_table(file)


def _tire_options(model, value):
    supplied = _object(value, "Tire settings")
    tires = {corner: TireSetup() for corner in CORNERS if f"{corner}_WC" in model.point_names}
    for corner, fields in supplied.items():
        fields = _object(fields, f"{corner} tire settings")
        if corner not in tires or fields.keys() - TireSetup.__dataclass_fields__.keys():
            raise ValueError(f"Invalid tire settings for {corner}.")
        setup = TireSetup(**{name: (_finite(value, f"{corner} {name}")
                                   if value is not None or name != "toe_deg" else None)
                            for name, value in fields.items()})
        if setup.width_mm <= 0 or setup.radius_mm <= 0:
            raise ValueError(f"{corner} tire width and radius must be greater than zero.")
        if abs(setup.camber_deg) >= 90 or (setup.toe_deg is not None and abs(setup.toe_deg) >= 90):
            raise ValueError(f"{corner} toe and camber must be between −90° and 90°.")
        tires[corner] = setup
    return tires


def _motion_options(model, value):
    saved = _object(value, "Motion settings")
    drives = [drive for drive in model.drives if not drive.locked]
    amplitudes = {drive.name: drive.amplitude_mm for drive in drives}
    profiles = {drive.name: drive.profile for drive in drives}
    options = {"duration": model.duration, "samples": model.samples,
               "amplitudes_mm": amplitudes, "profiles": profiles}
    if saved.keys() - options.keys():
        raise ValueError("Motion settings contain an unknown field.")
    for key in ("duration", "samples"):
        if key in saved:
            options[key] = _finite(saved[key], key.capitalize())
    amplitudes.update({name: _finite(amplitude, f"{name} amplitude") for name, amplitude in
                       _object(saved.get("amplitudes_mm", {}), "Drive amplitudes").items()})
    profiles.update(_object(saved.get("profiles", {}), "Drive profiles"))
    if any(not isinstance(profile, str) for profile in profiles.values()):
        raise ValueError("Drive profiles must be pulse, sine, or ramp.")
    motion = make_motion(model, **options)
    options["samples"] = len(motion.time)
    return options


def _build_project(filename, table, points, settings, mechanism, builtin=False):
    mounts = _object(settings.get("pushrod_mounts", {}), "Pushrod mounts")
    if any(not isinstance(mount, str) for mount in mounts.values()):
        raise ValueError("Pushrod mounts must be UCA, LCA, or UPRIGHT.")
    members = _read_deformable_members(settings.get("deformable_members", []))
    if mechanism is not None:
        mechanism = _json_object(mechanism, "Mechanism")
        mechanism = {"name": Path(filename).stem, **mechanism}
    with TemporaryDirectory() as directory:
        source = REFERENCE_CSV if builtin else Path(directory) / filename
        mechanism_path = None
        if mechanism is not None:
            mechanism_path = Path(directory) / "mechanism.json"
            mechanism_path.write_text(json.dumps(mechanism, allow_nan=False), encoding="utf-8")
        model = load_model(source, mechanism_path, points=points, pushrod_mounts=mounts,
                           deformable_members=members)
    tires = _tire_options(model, settings.get("tires", {}))
    motion = _motion_options(model, settings.get("motion", {}))
    return Project(filename, table, points, model, tires, motion, mechanism, builtin)


def _settings(project):
    return {"version": 1, "pushrod_mounts": dict(project.model.pushrod_mounts),
            "tires": {corner: asdict(setup) for corner, setup in project.tires.items()},
            "motion": project.motion,
            "deformable_members": [asdict(member) for member in project.model.deformable_members]}


def _install(project):
    global _project, _result, _ratio
    output = _describe(project)
    _project = project
    _result = _ratio = None
    return output


def _require_project():
    if _project is None:
        raise ValueError("Load a project first.")
    return _project


def _require_solution():
    project = _require_project()
    if _result is None:
        raise ValueError("Run a motion solve first.")
    return project, _result


def _load(text=None, filename=None, mechanism=None, example=None):
    if text is None:
        if example not in (None, "damper"):
            raise ValueError("Unknown example.")
        table = read_hardpoint_table(REFERENCE_CSV)
        settings = _read_settings(table)
        if example == "damper":
            settings["deformable_members"] = [
                {"name": f"DAMPER_{corner}", "a": {"xyz": [.02, side * .31, .37], "mount": corner},
                 "b": {"xyz": [.02, side * .15, .45], "mount": "CHASSIS"}}
                for corner, side in (("FL", 1), ("FR", -1))
            ]
        return _install(_build_project(REFERENCE_CSV.name, table, table.points, settings, mechanism, builtin=True))

    if not isinstance(text, str):
        raise ValueError("Project import requires CSV or JSON text.")
    filename = PureWindowsPath(filename or "project").name.replace(":", "_")
    if not filename or filename in (".", ".."):
        filename = "project"
    if filename.lower().endswith(".json") or text.lstrip("\ufeff \t\r\n").startswith("{"):
        data = _json_object(text, "Project")
        if "points" not in data:
            raise ValueError("Select a complete project JSON or select a mechanism JSON with its hardpoint CSV.")
        if type(data.get("version", 1)) is not int or data.get("version", 1) != 1:
            raise ValueError("The project JSON version must be 1.")
        if data.keys() - PROJECT_FIELDS - {"version"}:
            raise ValueError("Project JSON contains unknown fields.")
        points = _hardpoints(data.get("points"))
        with TemporaryDirectory() as directory:
            table = _table_from_points(points, Path(directory) / "hardpoints.csv")
        return _install(_build_project(filename, table, points, data, mechanism or data.get("mechanism")))

    with TemporaryDirectory() as directory:
        csv_path = Path(directory) / "input.csv"
        csv_path.write_bytes(text.encode("utf-8"))
        table = read_hardpoint_table(csv_path)
    settings = _read_settings(table)
    return _install(_build_project(filename, table, table.points, settings, mechanism or settings.get("mechanism")))


def _update(**changes):
    previous = _require_project()
    if changes.keys() - PROJECT_FIELDS:
        raise ValueError("The project update contains unknown fields.")
    settings = _json_ready(_settings(previous))
    points = _hardpoints(changes.get("points", previous.points))
    table = previous.table
    if tuple(table.points) != tuple(points):
        with TemporaryDirectory() as directory:
            table = _table_from_points(points, Path(directory) / "hardpoints.csv")
    mechanism = changes.get("mechanism", previous.mechanism)
    if mechanism != previous.mechanism:
        settings["pushrod_mounts"] = {}
        settings["motion"] = {}
        if mechanism is not None:
            settings["deformable_members"] = [member for member in settings["deformable_members"]
                                              if member["a"]["mount"] == member["b"]["mount"] == "CHASSIS"]
    settings.update(changes)
    return _install(_build_project(previous.filename, table, points, settings, mechanism, previous.builtin))


def _wheel_frames(project):
    model = project.model
    indices = {name: i for i, name in enumerate(model.point_names)}
    return {corner: WheelFrame.from_model(model, corner, indices, setup)
            for corner, setup in project.tires.items()}


def _describe(project):
    model = project.model
    wheels = []
    for corner, frame in _wheel_frames(project).items():
        setup = project.tires[corner]
        wheels.append({"corner": corner, "center": model.point_names[frame.center],
                       "normal": frame.reference_normal, "width": setup.width_mm / 1000,
                       "radius": setup.radius_mm / 1000, "width_mm": setup.width_mm, "radius_mm": setup.radius_mm})
    name = "Reference suspension" if model.reference_settings else model.name
    reference = asdict(model.reference_settings) if model.reference_settings else None
    if reference:
        reference["name"] = name
    return {**_settings(project), "filename": project.filename, "name": name,
            "points": project.points, "point_names": model.point_names, "reference": model.reference,
            "fixed": model.fixed, "links": [asdict(link) for link in model.links],
            "drives": [asdict(drive) for drive in model.drives], "internal_points": model.internal_points,
            "mechanism": project.mechanism, "reference_settings": reference, "wheels": wheels,
            "sweep_corners": sweep_corners(model), "has_front_axle": has_front_axle(model),
            "steering_corners": [corner for corner in CORNERS if has_steering(model, corner)],
            "damper_axles": damper_axles(model)}


def _solve():
    global _result, _ratio
    project = _require_project()
    result = solve(project.model, make_motion(project.model, **project.motion))
    interferences = find_interferences(project.model, result)
    frames = _wheel_frames(project)
    _result, _ratio = result, None
    return {"time": result.time, "position": result.position, "condition": result.condition,
            "interferences": [asdict(entry) for entry in interferences],
            "interference_tolerance_mm": INTERFERENCE_TOLERANCE_M * 1000,
            "wheel_normals": {corner: frame.normal(result.position) for corner, frame in frames.items()}}


def _signals(kind, names):
    project, result = _require_solution()
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise ValueError("Signal names must be a list of point, link, or damper names.")
    if kind == "damper":
        histories = damper_histories(project.model, result)
        missing = set(names) - histories.keys()
        if missing:
            raise ValueError(f"Unknown damper: {', '.join(sorted(missing))}.")
        return {name: asdict(histories[name]) for name in names}
    fields = {"point": ("position", "displacement", "velocity", "acceleration"),
              "rod": ("azimuth_deg", "elevation_deg", "angle_deg", "angular_velocity", "angular_acceleration")}
    if kind not in fields:
        raise ValueError("Select a point, link, or damper signal.")
    indices = {name: i for i, name in enumerate(result.point_names if kind == "point" else result.link_names)}
    data = {}
    for name in names:
        if name not in indices:
            raise ValueError(f"Unknown {kind}: {name}.")
        index = indices[name]
        signals = {field: getattr(result, field)[:, index] for field in fields[kind]}
        if kind == "rod":
            link = project.model.links[index]
            a, b = (result.point_names.index(point) for point in (link.a, link.b))
            length = np.linalg.norm(result.position[:, b] - result.position[:, a], axis=1)
            static_length = np.linalg.norm(project.model.reference[b] - project.model.reference[a])
            signals.update(length=length, compression=static_length - length)
        data[name] = signals
    return data


def _direction(value):
    # The desktop function normalizes axes and vectors and handles the sign conventions.
    if isinstance(value, str) and "," in value:
        return [_finite(component, "Direction component") for component in value.split(",")]
    return value


def _calculate_ratio(input_point, output_point, input_direction="z", output_direction="z", drive=None):
    global _ratio
    project, result = _require_solution()
    drive = drive or None
    if drive is not None and not any(item.name == drive and not item.locked for item in project.model.drives):
        raise ValueError("Select an unlocked drive to compute a Jacobian motion ratio.")
    _ratio = motion_ratio(result, input_point, output_point, _direction(input_direction),
                          _direction(output_direction), drive=drive)
    return asdict(_ratio)


def _limits(value, label):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"Specify lower and upper limits for {label}.")
    return tuple(_finite(limit, label) for limit in value)


def _suspension(corner, travel_limits, rack_limits=(0, 0)):
    project = _require_project()
    return asdict(sweep_suspension(project.model, corner, _limits(travel_limits, "wheel travel"),
                                  _limits(rack_limits, "rack displacement"), project.tires))


def _front_axle(heave_mm, roll_deg, rack_limits, wheelbase_mm=None):
    project = _require_project()
    wheelbase = None if wheelbase_mm is None else _finite(wheelbase_mm, "Wheelbase")
    return asdict(sweep_front_axle(project.model, _finite(heave_mm, "Heave"), _finite(roll_deg, "Roll"),
                                  _limits(rack_limits, "rack displacement"), wheelbase, project.tires))


def _damper_sweep(axle, mode, limits_mm, fixed_mm):
    project = _require_project()
    return asdict(sweep_dampers(project.model, axle, mode, _limits(limits_mm, mode),
                               _finite(fixed_mm, "Fixed displacement")))


def _project_json():
    project = _require_project()
    data = {**_settings(project), "points": project.points}
    if project.mechanism is not None:
        data["mechanism"] = project.mechanism
    return {"name": f"{Path(project.filename).stem}.json", "text": json.dumps(data, indent=2, ensure_ascii=False) + "\n"}


def _project_csv():
    project = _require_project()
    filename = f"{Path(project.filename).stem}.csv"
    with TemporaryDirectory() as directory:
        path = Path(directory) / "output.csv"
        saved = save_project(path, project.table, project.points, project.model.pushrod_mounts,
                             project.tires, project.motion, project.model.deformable_members, update_source=False)
    text = saved.content.decode("utf-8")
    if project.mechanism is not None:
        # The desktop CSV reader ignores additional project fields. Keep topology for browser round trips.
        column = _project_column(saved.headers)
        first, *rows = saved.rows
        settings = json.loads(first[column])
        settings["mechanism"] = project.mechanism
        first = (*first[:column], json.dumps(settings, ensure_ascii=False), *first[column + 1:])
        output = StringIO(newline="")
        if text.startswith("\ufeff"):
            output.write("\ufeff")
        writer = csv.writer(output, lineterminator="\r\n" if "\r\n" in text else "\n")
        writer.writerow(saved.headers)
        writer.writerows((first, *rows))
        text = output.getvalue()
    return {"name": filename, "text": text}


def _export(kind="all", ratio=None, include_ratio=True):
    project, result = _require_solution()
    names = {"point": "point_kinematics.csv", "rod": "rod_kinematics.csv",
             "ratio": "motion_ratio.csv", "damper": "damper_kinematics.csv"}
    if kind not in names and kind != "all":
        raise ValueError("Select a result type to download.")
    if ratio is not None:
        _calculate_ratio(**_object(ratio, "Motion ratio settings"))
    if kind == "ratio" and _ratio is None:
        raise ValueError("Calculate a motion ratio before downloading it.")
    with TemporaryDirectory() as directory:
        folder = Path(directory)
        paths = () if kind == "damper" else export_results(folder, result, _ratio if include_ratio else None)
        if kind in ("damper", "all"):
            histories = damper_histories(project.model, result)
            if histories:
                path = folder / names["damper"]
                _write_csv(path, ("time_s", "damper", "compression_m", "velocity_m_s", "acceleration_m_s2"),
                           ((time, name, history.compression[i], history.velocity[i], history.acceleration[i])
                            for i, time in enumerate(result.time) for name, history in histories.items()))
                paths += (path,)
            elif kind == "damper":
                raise ValueError("Add and solve a deformable member before downloading damper results.")
        return [{"name": path.name, "text": path.read_bytes().decode("utf-8")}
                for path in paths if kind == "all" or path.name == names[kind]]


OPERATIONS = {"load": _load, "update": _update, "solve": _solve, "signals": _signals,
              "ratio": _calculate_ratio, "suspension": _suspension, "front_axle": _front_axle,
              "damper_sweep": _damper_sweep, "project_json": _project_json, "project_csv": _project_csv,
              "export": _export}


def _json_ready(value):
    # An undefined angle or tangent is represented as null for browser tables and charts.
    if isinstance(value, np.ndarray):
        return _json_ready(value.tolist())
    if isinstance(value, np.generic):
        return _json_ready(value.item())
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


_DIAGNOSTICS = (
    (r"硬點 CSV 是空的。|硬點 CSV 沒有座標列。", r"The hardpoint CSV has no data rows."),
    (r"硬點 CSV 有重複欄名。", r"The hardpoint CSV has duplicate column names."),
    (r"硬點 CSV 需要 name.*。", r"Use name or point plus x_m, y_m, z_m or x_mm, y_mm, z_mm."),
    (r"硬點 CSV 第 (\d+) 列的欄數不一致。", r"CSV row \1 has an unexpected column count."),
    (r"硬點 CSV 第 (\d+) 列缺少點名。", r"CSV row \1 is missing a point name."),
    (r"硬點 CSV 有重複點名 (.+)。", r"The hardpoint CSV repeats point \1."),
    (r"硬點 CSV 只能包含一筆專案設定。", r"The CSV may contain only one jacobian_project entry."),
    (r"硬點 CSV 的專案設定格式不正確。", r"The CSV project settings are invalid. Use project version 1."),
    (r"硬點 CSV 的變形元件端點格式不正確。", r"Each deformable endpoint needs xyz and mount."),
    (r"硬點 CSV 的變形元件設定格式不正確。", r"Deformable members must be a list of entries with name, a, and b."),
    (r"硬點 (.+) 不符合標準懸吊點名規則。請參考硬點命名表。",
     r"Hardpoint \1 does not match the standard naming scheme. Check the hardpoint guide or supply a matching mechanism JSON."),
    (r"硬點 (.+) 有重複定義。", r"Multiple rows define the canonical hardpoint \1."),
    (r"(.+) 缺少懸吊硬點 (.+)。", r"\1 is missing hardpoints \2."),
    (r"(.+) 的轉向拉桿需要同時提供 RACK 與 TOE。", r"The \1 tie rod needs both RACK and TOE hardpoints."),
    (r"(.+) 的 FORWARD 需要搭配輪心 WC。", r"The \1 FORWARD marker requires a wheel center (WC)."),
    (r"前齒條需要 FL_RACK 的 Y 大於 FR_RACK，以定義向左為正的輸入。",
     r"FL_RACK Y must be greater than FR_RACK Y so leftward rack travel is positive."),
    (r"(.+) 的推桿需提供一個 PUSH_OUT.+。", r"\1 needs one PUSH_OUT variant, PUSH_IN, ROCKER_A, and ROCKER_B."),
    (r"(.+) 的推桿外端需選擇上 A 臂、下 A 臂或羊角。", r"The \1 pushrod must mount to UCA, LCA, or UPRIGHT."),
    (r"(.+) 的推桿外端要裝在羊角上，需提供輪心 WC 或轉向拉桿外端 TOE 定義羊角姿態。",
     r"The \1 upright mount needs WC or TOE to define its orientation."),
    (r"(.+) 沒有可設定的推桿。", r"No pushrod is defined for \1."),
    (r"沒有可建立機構的輪位。", r"The hardpoints define no complete suspension corner."),
    (r"此檔案是基準車輛參數 JSON。.+", r"This is a vehicle parameter file. Import a project or a matching desktop mechanism JSON."),
    (r"機構 JSON 需要 schema_version 1。", r"The mechanism JSON must have schema_version 1."),
    (r"機構 JSON 有重複固定點。", r"The mechanism lists a fixed point more than once."),
    (r"機構 JSON 有重複的剛體附著點。", r"The mechanism defines an attachment point more than once."),
    (r"機構 JSON 有重複(?:桿件|驅動)名稱。", r"The mechanism contains duplicate link or drive names."),
    (r"自訂機構的推桿接法由機構 JSON 定義。", r"Set custom pushrod mounts in the mechanism JSON."),
    (r"(.+) 使用未匯入的硬點 (.+)。", r"\1 refers to hardpoint \2, which does not exist in the imported data."),
    (r"桿件 (.+) 的兩端必須不同。", r"Link \1 must connect two different hardpoints."),
    (r"桿件 (.+) 的 constrained 需為布林值。", r"The constrained field on link \1 must be true or false."),
    (r"驅動 (.+) 的兩個點必須不同。", r"Drive \1 must reference two different hardpoints."),
    (r"剛體附著 (.+) 需要三個不同的基準點，且不能包含自身。",
     r"Attachment \1 needs three distinct anchors, each different from the attached point."),
    (r"(.+) 的 profile 需為 pulse、sine 或 ramp。locked 需為布林值，鎖定時振幅為零。",
     r"Drive \1 needs a pulse, sine, or ramp profile and a Boolean locked field. Set a locked drive's amplitude to zero."),
    (r"(.+) 的行程形式需為 pulse、sine 或 ramp。", r"The \1 profile must be pulse, sine, or ramp."),
    (r"無法修改驅動 (.+) 的振幅或行程形式。", r"The amplitude or profile of drive \1 cannot be overridden."),
    (r"桿件名稱 (.+) 已使用。", r"The link name \1 is already in use."),
    (r"端點名稱 (.+) 已使用。請更換元件名稱。", r"The endpoint \1 already exists. Rename the deformable member."),
    (r"(.+) 選擇的搖臂不存在。", r"The rocker selected for endpoint \1 does not exist."),
    (r"(.+) 的靜態長度須大於零且為有限數值。", r"The reference length of \1 must be finite and greater than zero."),
    (r"工況時間需大於零，取樣數需為 2 到 5001 的整數。",
     r"Use a positive duration and a whole sample count between 2 and 5001."),
    (r"工況設定超出可計算的時間或振幅範圍。", r"The motion duration or amplitude is outside the numerical range."),
    (r"剛體附著點及其三個基準點須各自不同。", r"The attached hardpoint and its three anchors must be distinct."),
    (r"剛體附著點 (.+) 的三個基準點共線或座標範圍無效。",
     r"The anchors for attachment \1 are collinear or outside the numerical range."),
    (r"桿件的靜態端點重合或距離無效。", r"A link has coincident endpoints or an invalid reference length."),
    (r"參考姿態的 Jacobian 秩不足，(\d+) 個座標方向未受約束或在奇異位置。請檢查固定點、桿件及驅動方向。",
     r"The reference Jacobian is rank-deficient by \1 coordinate(s). Check the fixed points, links, and drive directions."),
    (r"Jacobian 接近奇異，無法唯一決定此姿態的運動。", r"The Jacobian is near singular and cannot determine a unique motion."),
    (r"位置迭代未收斂，指定行程可能與桿長約束不相容。",
     r"The position solve did not converge. The requested travel may be incompatible with the link lengths."),
    (r"單次驅動位移差超出續接求解範圍（(.+) mm）。請縮短行程或增加時間採樣點。",
     r"Drive displacement changed by more than \1 mm between samples. Reduce travel or increase the sample count."),
    (r"(?:輸入或運動|節點位置或桿件角運動|加速度|速度)超出可計算範圍。",
     r"The motion exceeds the numerical range of the solver."),
    (r"指定速度與剛體幾何約束不相容。", r"The prescribed velocity is incompatible with the rigid geometry constraints."),
    (r"指定加速度與剛體幾何約束不相容。", r"The prescribed acceleration is incompatible with the rigid geometry constraints."),
    (r"(?:輪跳|齒條位移|ROLL|HEAVE)下限須小於上限，且兩者須為有限數值。",
     r"Specify finite sweep limits with the lower value smaller than the upper value."),
    (r"齒條位移下限須小於或等於上限，且兩者須為有限數值。",
     r"Use finite rack limits. The lower limit must be less than or equal to the upper limit."),
    (r"懸吊特性掃描需要標準懸吊模型及所選輪位的輪心 WC。",
     r"Single-wheel analysis requires a standard suspension with WC at the selected corner."),
    (r"前軸掃描需要 FL、FR 兩輪的 WC、RACK 與 TOE。", r"Front steering analysis requires WC, RACK, and TOE for both FL and FR."),
    (r"Heave 須為有限數值；Roll 須介於 −90° 與 90° 之間。", r"Use a finite heave and a roll angle between −90° and 90°."),
    (r"前輪需要非共線的羊角硬點，以定義輪向。", r"The front uprights need noncollinear hardpoints to define wheel orientation."),
    (r"前軸左右輪心需要有效的 Y 向間距。", r"The front wheel centers need valid spacing along Y."),
    (r"軸距須為大於零的有限數值。", r"Wheelbase must be finite and greater than zero."),
    (r"避震器行程掃描需要所選車軸的左右輪心 WC。", r"The selected axle needs left and right wheel centers (WC)."),
    (r"請先新增變形元件。", r"Add a deformable member first."),
    (r"請選擇 ROLL 或 HEAVE，並輸入有限的固定值。", r"Choose ROLL or HEAVE and specify a finite fixed displacement."),
    (r"ROLL 掃描使用位移差的大小，下限須大於或等於零。", r"The lower ROLL limit must be zero or greater."),
    (r"方向 .+ 須為 X、Y、Z 其中一軸.*。|方向須為非零的有限三維向量。",
     r"Use a signed axis such as -z or a finite nonzero vector such as 1,0,2."),
    (r"找不到節點 (.+)。", r"Point \1 is not in the solution."),
    (r"(.+?)\s*需要有限數值。", r"\1 must be a finite number."),
    (r"(.+?)\s*需要三個 XYZ 數值。", r"\1 requires three XYZ coordinates."),
    (r"(.+?)\s*需要非零方向。", r"\1 needs a nonzero direction."),
    (r"(.+?)\s*需要名稱。", r"\1 needs a name."),
    (r"(.+?)\s*必須是 JSON 物件。", r"\1 must be a JSON object."),
    (r"(.+?)\s*必須是 JSON 陣列。", r"\1 must be a JSON array."),
    (r"(.+?) 有未知欄位 (.+)。", r"\1 contains unknown fields: \2."),
    (r"(.+?) 缺少 (.+)。", r"\1 is missing \2."),
)

_LABELS = {"硬點": "Hardpoint", " 的方向": " direction", " 的振幅": " amplitude", " 的 X": " X",
           " 的 Y": " Y", " 的 Z": " Z", "前齒條軸向": "Front rack axis", "機構": "Mechanism",
           "剛體附著": "Attachment", "桿件": "Link", "驅動": "Drive", "固定點": "Fixed point",
           "變形元件": "Deformable member", "輪跳": "wheel travel", "齒條": "rack travel",
           "左側 ROLL": "Left ROLL", "右側 ROLL": "Right ROLL", "前左輪跳": "left-front travel",
           "前右輪跳": "right-front travel", " 的輪心至 FORWARD 水平方向": " horizontal WC-to-FORWARD direction",
           "工況時間": "Duration", "取樣數": "Sample count"}
_LABEL_PATTERN = re.compile("|".join(re.escape(label) for label in sorted(_LABELS, key=len, reverse=True)))


def _english_error(message):
    prefix, separator, cause = message.partition("：")
    if separator:
        return f"{_LABEL_PATTERN.sub(lambda match: _LABELS[match.group()], prefix).replace('、', ', ')}: {_english_error(cause)}"
    for pattern, translation in _DIAGNOSTICS:
        match = re.fullmatch(pattern, message)
        if match:
            return _LABEL_PATTERN.sub(lambda item: _LABELS[item.group()], match.expand(translation))
    if re.search(r"[\u3400-\u9fff]", message):
        return "The kinematics engine could not complete this operation. Check the input geometry and settings."
    return message


def dispatch(operation, payload):
    """Handle one JSON message from the browser worker."""
    try:
        if operation not in OPERATIONS:
            raise ValueError(f"Unknown operation {operation!r}.")
        response = OPERATIONS[operation](**_json_object(payload, "Request"))
        return json.dumps(_json_ready(response), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except (ValueError, KeyError, TypeError, OverflowError, csv.Error, OSError, np.linalg.LinAlgError) as error:
        raise ValueError(_english_error(str(error))) from None
