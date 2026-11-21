"""Read and save workbench projects as CSV or JSON."""

import csv
import json
from codecs import BOM_UTF8
from dataclasses import asdict, dataclass, replace
from io import StringIO
from math import isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile

from .model import (
    CORNERS, DeformableEnd, DeformableMember, _name, _number, load_model, make_motion, read_hardpoint_table,
)
from .wheel_frame import TireSetup


PROJECT_COLUMN = "jacobian_project"
PROJECT_SECTIONS = ("pushrod_mounts", "tires", "motion")
JSON_PROJECT_FIELDS = {*PROJECT_SECTIONS, "version", "points", "deformable_members", "mechanism"}


@dataclass(frozen=True)
class JsonProject:
    points: dict[str, tuple[float, float, float]]
    content: bytes
    mechanism: dict | None


def _project_column(headers):
    return next((i for i, name in enumerate(headers) if name.strip().lower() == PROJECT_COLUMN), None)


def _read_settings(table):
    column = _project_column(table.headers)
    if column is None:
        return {}
    values = [row[column] for row in table.rows if row[column].strip()]
    if not values:
        return {}
    if len(values) != 1:
        raise ValueError("硬點 CSV 只能包含一筆專案設定。")
    settings = json.loads(values[0])
    if (not isinstance(settings, dict) or settings.get("version") != 1
            or any(not isinstance(settings.get(key, {}), dict) for key in PROJECT_SECTIONS)):
        raise ValueError("硬點 CSV 的專案設定格式不正確。")
    return settings


def _read_deformable_end(end, source):
    if (not isinstance(end, dict) or set(end) != {"xyz", "mount"}
            or not isinstance(end["xyz"], list) or not isinstance(end["mount"], str)):
        raise ValueError(f"{source} 的變形元件端點格式不正確。")
    return DeformableEnd(tuple(end["xyz"]), end["mount"])


def _read_deformable_members(members, source="硬點 CSV"):
    if not isinstance(members, list):
        raise ValueError(f"{source} 的變形元件設定格式不正確。")
    result = []
    for member in members:
        if not isinstance(member, dict) or set(member) != {"name", "a", "b"}:
            raise ValueError(f"{source} 的變形元件設定格式不正確。")
        result.append(DeformableMember(member["name"], _read_deformable_end(member["a"], source),
                                       _read_deformable_end(member["b"], source)))
    return tuple(result)


def _decode_json_object(content, label):
    try:
        data = json.loads(content.decode("utf-8-sig"))
    except UnicodeError as error:
        raise ValueError(f"{label} 需要使用 UTF-8 編碼。") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"{label} 格式不正確（第 {error.lineno} 行，第 {error.colno} 欄）。") from error
    if not isinstance(data, dict):
        raise ValueError(f"{label} 必須是 JSON 物件。")
    return data


def _read_json_project(path, mechanism):
    content = path.read_bytes()
    data = _decode_json_object(content, "專案 JSON")
    if "points" not in data:
        raise ValueError("專案 JSON 缺少 points。請選擇網頁的 Download JSON 專案檔。")
    version = data.get("version", 1)
    if type(version) is not int or version != 1:
        raise ValueError("專案 JSON 需要 version 1。")
    if data.keys() - JSON_PROJECT_FIELDS or any(not isinstance(data.get(key, {}), dict) for key in PROJECT_SECTIONS):
        raise ValueError("專案 JSON 的設定格式不正確。")
    if not isinstance(data["points"], dict) or not data["points"]:
        raise ValueError("專案 JSON 的 points 需要至少一個硬點。")
    points = {}
    for raw_name, xyz in data["points"].items():
        name = _name(raw_name, "硬點")
        if name in points:
            raise ValueError(f"專案 JSON 有重複點名 {name}。")
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError(f"硬點 {name} 需要三個 XYZ 座標，單位為公尺。")
        points[name] = tuple(_number(value, f"硬點 {name} 的 {axis} ") for axis, value in zip("XYZ", xyz))

    if mechanism is None:
        mechanism = data.get("mechanism")
        if mechanism is not None and not isinstance(mechanism, dict):
            raise ValueError("專案 JSON 的 mechanism 必須是 JSON 物件。")
    elif not isinstance(mechanism, dict):
        mechanism_path = Path(mechanism)
        mechanism = {"name": mechanism_path.stem, **_decode_json_object(mechanism_path.read_bytes(), "機構 JSON")}
    return JsonProject(points, content, mechanism), data


def load_project(path, mechanism=None):
    path = Path(path)
    if path.suffix.lower() == ".json":
        table, settings = _read_json_project(path, mechanism)
        mechanism = table.mechanism
        source = "專案 JSON"
    else:
        try:
            table = read_hardpoint_table(path)
        except (csv.Error, UnicodeError) as error:
            raise ValueError(f"無法讀取 UTF-8 硬點 CSV。{error}") from error
        settings = _read_settings(table)
        source = "硬點 CSV"
    mounts = settings.get("pushrod_mounts", {})
    if any(not isinstance(mount, str) for mount in mounts.values()):
        raise ValueError(f"{source} 的推桿接法格式不正確。")
    members = _read_deformable_members(settings.get("deformable_members", []), source)
    model = load_model(path, mechanism, points=table.points, pushrod_mounts=mounts, deformable_members=members)
    tires = {corner: TireSetup() for corner in CORNERS if f"{corner}_WC" in model.point_names}
    drives = [drive for drive in model.drives if not drive.locked]
    motion = {"duration": model.duration, "samples": model.samples,
              "amplitudes_mm": {drive.name: drive.amplitude_mm for drive in drives},
              "profiles": {drive.name: drive.profile for drive in drives}}
    try:
        for corner, fields in settings.get("tires", {}).items():
            setup = TireSetup(**fields)
            if (corner not in tires or not all(isfinite(value) and value > 0.
                                              for value in (setup.width_mm, setup.radius_mm))
                    or (setup.toe_deg is not None and not abs(setup.toe_deg) < 90.)
                    or not abs(setup.camber_deg) < 90.):
                raise ValueError(f"{corner} 的輪胎設定無效。")
            tires[corner] = setup
        saved_motion = settings.get("motion", {})
        duration = saved_motion.get("duration", model.duration)
        samples = saved_motion.get("samples", model.samples)
        if (type(duration) not in (int, float) or type(samples) is not int
                or saved_motion.keys() - motion.keys()):
            raise ValueError(f"{source} 的輪跳與轉向設定格式不正確。")
        motion.update(duration=duration, samples=samples)
        for key in ("amplitudes_mm", "profiles"):
            saved = saved_motion.get(key, {})
            if not isinstance(saved, dict):
                raise ValueError(f"{source} 的輪跳與轉向設定格式不正確。")
            motion[key].update(saved)
        if saved_motion:
            make_motion(model, **motion)
    except (TypeError, AttributeError, OverflowError) as error:
        raise ValueError(f"{source} 的輪胎或輪跳設定格式不正確。") from error
    return table, model, tires, motion


def save_project(path, table, points, mounts, tires, motion, deformable_members=(), *, update_source):
    """Save edits in the source format and preserve additional CSV fields."""
    path = Path(path)
    settings = {"version": 1, "pushrod_mounts": dict(mounts),
                "tires": {corner: asdict(setup) for corner, setup in tires.items()}, "motion": motion}
    if deformable_members:
        settings["deformable_members"] = [asdict(member) for member in deformable_members]
    encoding = "utf-8-sig" if table.content.startswith(BOM_UTF8) else "utf-8"
    if isinstance(table, JsonProject):
        if path.suffix.lower() != ".json":
            raise ValueError("請使用 .json 副檔名儲存 JSON 專案。")
        settings["points"] = points
        if table.mechanism is not None:
            settings["mechanism"] = table.mechanism
        text = json.dumps(settings, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        content = text.encode(encoding)
        _write_project(path, table.content, content, update_source, "JSON")
        return replace(table, points=points, content=content)
    if path.suffix.lower() == ".json":
        raise ValueError("請使用 .csv 副檔名儲存 CSV 專案。")

    payload = json.dumps(settings, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    column = _project_column(table.headers)
    headers = table.headers if column is not None else (*table.headers, PROJECT_COLUMN)
    rows = []
    for name, original_row in zip(table.points, table.rows):
        row = list(original_row)
        for index, old, new in zip(table.coordinate_columns, table.points[name], points[name]):
            if old != new:
                row[index] = str(new / table.scale)
        metadata = payload if not rows else ""
        if column is None:
            row.append(metadata)
        else:
            row[column] = metadata
        rows.append(tuple(row))

    output = StringIO(newline="")
    line_ending = "\r\n" if b"\r\n" in table.content else "\n"
    writer = csv.writer(output, lineterminator=line_ending)
    writer.writerow(headers)
    writer.writerows(rows)
    content = output.getvalue().encode(encoding)
    _write_project(path, table.content, content, update_source, "CSV")
    return replace(table, points=points, headers=headers, rows=tuple(rows), content=content)


def _write_project(path, original, content, update_source, format_name):
    temporary_path = None
    try:
        with NamedTemporaryFile(dir=path.parent, suffix=".tmp", delete=False) as handle:
            temporary_path = Path(handle.name)
            handle.write(content)
        if update_source and path.read_bytes() != original:
            raise ValueError(f"磁碟上的 {format_name} 已變更。請重新匯入後再編輯。")
        temporary_path.replace(path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
