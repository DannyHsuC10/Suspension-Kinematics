"""Read and save the workbench project in a hardpoint CSV."""

import csv
import json
from codecs import BOM_UTF8
from dataclasses import asdict, replace
from io import StringIO
from math import isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile

from .model import CORNERS, DeformableEnd, DeformableMember, load_model, make_motion, read_hardpoint_table
from .wheel_frame import TireSetup


PROJECT_COLUMN = "jacobian_project"
PROJECT_SECTIONS = ("pushrod_mounts", "tires", "motion")


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


def _read_deformable_end(end):
    if (not isinstance(end, dict) or set(end) != {"xyz", "mount"}
            or not isinstance(end["xyz"], list) or not isinstance(end["mount"], str)):
        raise ValueError("硬點 CSV 的變形元件端點格式不正確。")
    return DeformableEnd(tuple(end["xyz"]), end["mount"])


def _read_deformable_members(members):
    if not isinstance(members, list):
        raise ValueError("硬點 CSV 的變形元件設定格式不正確。")
    result = []
    for member in members:
        if not isinstance(member, dict) or set(member) != {"name", "a", "b"}:
            raise ValueError("硬點 CSV 的變形元件設定格式不正確。")
        result.append(DeformableMember(member["name"], _read_deformable_end(member["a"]),
                                       _read_deformable_end(member["b"])))
    return tuple(result)


def load_project(path, mechanism=None):
    try:
        table = read_hardpoint_table(path)
    except (csv.Error, UnicodeError) as error:
        raise ValueError(f"無法讀取 UTF-8 硬點 CSV。{error}") from error
    settings = _read_settings(table)
    mounts = settings.get("pushrod_mounts", {})
    if any(not isinstance(mount, str) for mount in mounts.values()):
        raise ValueError("硬點 CSV 的推桿接法格式不正確。")
    members = _read_deformable_members(settings.get("deformable_members", []))
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
            raise ValueError("硬點 CSV 的輪跳與轉向設定格式不正確。")
        motion.update(duration=duration, samples=samples)
        for key in ("amplitudes_mm", "profiles"):
            saved = saved_motion.get(key, {})
            if not isinstance(saved, dict):
                raise ValueError("硬點 CSV 的輪跳與轉向設定格式不正確。")
            motion[key].update(saved)
        if saved_motion:
            make_motion(model, **motion)
    except (TypeError, AttributeError, OverflowError) as error:
        raise ValueError("硬點 CSV 的輪胎或輪跳設定格式不正確。") from error
    return table, model, tires, motion


def save_project(path, table, points, mounts, tires, motion, deformable_members=(), *, update_source):
    """Update the current hardpoints and settings without dropping other CSV fields."""
    path = Path(path)
    settings = {"version": 1, "pushrod_mounts": dict(mounts),
                "tires": {corner: asdict(setup) for corner, setup in tires.items()}, "motion": motion}
    if deformable_members:
        settings["deformable_members"] = [asdict(member) for member in deformable_members]
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
    encoding = "utf-8-sig" if table.content.startswith(BOM_UTF8) else "utf-8"
    content = output.getvalue().encode(encoding)

    temporary_path = None
    try:
        with NamedTemporaryFile(dir=path.parent, suffix=".tmp", delete=False) as handle:
            temporary_path = Path(handle.name)
            handle.write(content)
        if update_source and path.read_bytes() != table.content:
            raise ValueError("磁碟上的 CSV 已變更。請重新匯入後再編輯。")
        temporary_path.replace(path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    return replace(table, points=points, headers=headers, rows=tuple(rows), content=content)
