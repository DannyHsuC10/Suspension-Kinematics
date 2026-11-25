"""Desktop workbench for importing, solving and inspecting kinematics."""

from pathlib import Path
from queue import Empty, SimpleQueue
from threading import Thread
from time import perf_counter
import tkinter as tk
from tkinter import filedialog, font, messagebox, ttk

import numpy as np

from .geometry_editor import DeformableSettings, GeometrySettings
from .interference import INTERFERENCE_TOLERANCE_M, find_interferences
from .model import EXAMPLES, load_model, make_motion
from .project import JsonProject, load_project, save_project
from .results import damper_histories, export_results, motion_ratio
from .solver import solve
from .suspension import (
    damper_axles, has_front_axle, has_steering, sweep_corners, sweep_dampers, sweep_front_axle, sweep_suspension,
)
from .viewport import MechanismViewport
from .wheel_frame import TireSetup


COLORS = ("#49d9c0", "#ffbc70", "#89aaff", "#ef9bd7")
COMPARISON_DASH = (6, 4)
BACKGROUND = "#edf2f6"
INK = "#132c40"
MUTED = "#496376"
FONT = "Microsoft JhengHei UI"
AXES = ("X", "Y", "Z")
DISPLACEMENT_AXES = ("ΔX", "ΔY", "ΔZ")
TIME_LABEL = "時間 (s)"
PATH_RATE = "沿時間歷程"
PROFILE_LABELS = {"pulse": "往返", "sine": "正弦", "ramp": "線性"}
CORNER_LABELS = {"FL": "左前", "FR": "右前", "RL": "左後", "RR": "右後"}
DEFAULT_HARDPOINTS = EXAMPLES / "hardpoints.csv"
CSV_FILE_TYPES = (("硬點表 CSV", "*.csv"), ("所有檔案", "*.*"))
PROJECT_FILE_TYPES = (("懸吊專案 CSV / JSON", ("*.csv", "*.json")), *CSV_FILE_TYPES)
POINT_FIELDS = {
    "位置": ("position", "mm", None),
    "相對位移": ("displacement", "mm", "總位移"),
    "速度": ("velocity", "mm/s", "合速度"),
    "加速度": ("acceleration", "mm/s²", "合加速度"),
}
ROD_FIELDS = ("桿軸角度", "角速度", "角加速度", "長度", "軸向壓縮量")
DAMPER_HISTORY_FIELDS = {
    "壓縮量": ("compression", "mm", "靜態長度減去當前長度。正值為壓縮，負值為伸長。"),
    "壓縮速度": ("velocity", "mm/s", "正值表示正在縮短，負值表示正在伸長。"),
    "壓縮加速度": ("acceleration", "mm/s²", "正值表示壓縮速度增加，負值表示壓縮速度減少。"),
}
DAMPER_SWEEP_FIELDS = ("伸縮量 (mm)", "左右差距 (%)")
RATIO_FIELDS = ("有號運動比", "理想機械利益")
SWEEP_FIELDS = {
    "外傾角 Camber": ("camber_deg", "deg", "正值表示輪胎頂端朝外。靜態外傾可在硬點與輪胎設定調整。"),
    "Camber gain": ("camber_gain", "deg/mm", "固定齒條位置，每上壓 1 mm 的局部外傾角變化。"),
    "前束角 Toe": ("toe_deg", "deg", "正值表示 toe-in，包含齒條轉向及輪跳造成的前束角。"),
    "Bump steer": ("bump_steer_deg", "deg", "前束角相對於同一齒條位置、零輪跳時的變化。正值表示 toe-in 增加。"),
    "Bump steer 變化率": ("bump_steer_rate", "deg/mm", "固定齒條位置，每上壓 1 mm 的局部前束角變化。"),
    "輪距": ("track_mm", "mm", "所選輪位同軸左右輪心的 Y 向間距。"),
    "軸距": ("wheelbase_mm", "mm", "所選輪位同側前後輪心的 X 向間距。"),
}
AXLE_FIELDS = {
    "左右前輪轉角": ("steer_deg", "deg", "相對同工況齒條零位，左右前輪皆以向左轉為正。"),
    "左右前輪絕對航向": ("yaw_deg", "deg", "以車輛 +X 為 0°，朝 +Y（向左）為正，包含靜態定位。"),
    "左右輪轉角差": ("difference_deg", "deg", "左前輪轉角減右前輪轉角。"),
    "阿克曼率": ("ackermann_pct", "%", "理想阿克曼為 100%，平行轉向為 0%。直行及不適用的樣本留白。"),
}
CAMERA_VIEWS = {"3D 斜視": "iso", "前視": "front", "側視": "side", "上視": "top"}
PLAYBACK_RATES = {"0.25×": .25, "0.5×": .5, "1×": 1., "2×": 2.}


def number(value):
    if not np.isfinite(value):
        return "—"
    return f"{value:.7g}" if value != 0 else "0"


def read_direction(text):
    text = text.strip().lower()
    if text in {"x", "y", "z", "+x", "+y", "+z", "-x", "-y", "-z"}:
        return text
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("投影方向請輸入 X、Y、Z、負軸，或三個向量分量（例如 1, 0, 2）。")
    try:
        return tuple(float(part) for part in parts)
    except ValueError as error:
        raise ValueError("投影方向的向量分量須為數字（例如 1, 0, 2）。") from error


def closest_sample(time, target):
    index = int(np.searchsorted(time, target))
    if index == len(time) or (index > 0 and target - time[index - 1] < time[index] - target):
        index -= 1
    return index


def _solve_with_interferences(model, motion):
    result = solve(model, motion)
    return result, find_interferences(model, result)


class SignalPlot(tk.Canvas):
    def __init__(self, parent, on_seek, x_label):
        super().__init__(parent, width=440, height=300, background="#102534", highlightthickness=0)
        self.on_seek = on_seek
        self.x_label = x_label
        self.x_values = np.empty(0)
        self.series = ()
        self.comparison = ()
        self.unit = ""
        self.index = None
        self.bounds = None
        self.bind("<Configure>", lambda _event: self.draw())
        self.bind("<Button-1>", self.seek)
        self.bind("<B1-Motion>", self.seek)

    def set_data(self, x_values, series, unit, *, comparison=()):
        self.x_values = x_values
        self.series = series
        self.comparison = comparison
        self.unit = unit
        self.index = None
        self.draw()

    def label(self, x, y, label, *, anchor="center", fill="#c0d2df", size=9):
        return self.create_text(x, y, text=label, anchor=anchor, fill=fill, font=(FONT, size))

    def draw(self):
        self.delete("all")
        self.bounds = None
        width, height = self.winfo_width(), self.winfo_height()
        if width < 240 or height < 130:
            return
        if not len(self.x_values):
            self.label(width / 2, height / 2, "目前沒有可顯示的曲線。", size=11)
            return
        left, right, top, bottom = 91, width - 28, 58, height - 55
        first, last = self.x_values[[0, -1]]
        if first == last:
            first, last = first - 0.5, last + 0.5
        ranges = [values[np.isfinite(values)] for _, values in (*self.series, *self.comparison)]
        ranges = [values for values in ranges if len(values)]
        if not ranges:
            self.label(width / 2, height / 2, "目前工況下沒有可用數值。", size=11)
            return
        low = min(values.min() for values in ranges)
        high = max(values.max() for values in ranges)
        margin = 0.07 * (high - low) if high > low else max(abs(high) * 0.07, 1e-3)
        low, high = low - margin, high + margin
        self.bounds = (left, right, first, last, top, bottom, low, high)

        def pixel_x(t):
            return left + (right - left) * (t - first) / (last - first)

        def pixel_y(v):
            return bottom - (bottom - top) * (v - low) / (high - low)

        for fraction in np.linspace(0, 1, 6):
            x = left + fraction * (right - left)
            y = bottom - fraction * (bottom - top)
            self.create_line(x, top, x, bottom, fill="#344958")
            self.create_line(left, y, right, y, fill="#344958")
            self.label(x, bottom + 14, f"{first + fraction * (last - first):.4g}")
            self.label(left - 10, y, f"{low + fraction * (high - low):.4g}", anchor="e")
        self.label(left, 42, self.unit, anchor="w", fill="#f0f8fd", size=10)
        self.label((left + right) / 2, bottom + 36, self.x_label)
        if low < 0 < high:
            zero = pixel_y(0)
            self.create_line(left, zero, right, zero, fill="#668092")

        legend_x = right
        for (label, _), color in reversed(tuple(zip(self.series, COLORS))):
            legend = self.label(legend_x, 22, label, anchor="e", fill=color, size=10)
            label_left = self.bbox(legend)[0]
            self.create_line(label_left - 20, 22, label_left - 6, 22, fill=color, width=2)
            legend_x = label_left - 26
        for series, dash in ((self.series, ()), (self.comparison, COMPARISON_DASH)):
            for (_, values), color in zip(series, COLORS):
                edges = np.flatnonzero(np.diff(np.r_[False, np.isfinite(values), False]))
                for start, stop in zip(edges[::2], edges[1::2]):
                    xs = pixel_x(self.x_values[start:stop])
                    ys = pixel_y(values[start:stop])
                    if stop - start == 1:
                        x, y = xs[0], ys[0]
                        self.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)
                    else:
                        coords = np.column_stack((xs, ys)).ravel().tolist()
                        self.create_line(*coords, width=4 if dash else 2, fill=color, dash=dash)

        self.draw_cursor()

    def set_index(self, index):
        if self.index != index:
            self.index = index
            self.draw_cursor()

    def draw_cursor(self):
        self.delete("playhead")
        if self.index is None or self.bounds is None:
            return
        left, right, first, last, top, bottom, low, high = self.bounds
        x = left + (right - left) * (self.x_values[self.index] - first) / (last - first)
        self.create_line(x, top, x, bottom, fill="#e4eef3", dash=(3, 3), tags="playhead")
        for series, radius in ((self.series, 4), (self.comparison, 6)):
            for (_, values), color in zip(series, COLORS):
                value = values[self.index]
                if np.isfinite(value):
                    y = bottom - (bottom - top) * (value - low) / (high - low)
                    fill, outline = (color, "#102534") if radius == 4 else ("", color)
                    self.create_oval(x - radius, y - radius, x + radius, y + radius,
                                     fill=fill, outline=outline, tags="playhead")

    def seek(self, event):
        if self.bounds is None:
            return
        left, right, first, last = self.bounds[:4]
        fraction = min(1, max(0, (event.x - left) / (right - left)))
        self.on_seek(closest_sample(self.x_values, first + fraction * (last - first)))


class SignalView(ttk.Frame):
    def __init__(self, parent, on_seek, x_label=TIME_LABEL):
        super().__init__(parent, padding=(12, 10))
        self.on_seek = on_seek
        self.x_label = x_label
        self.controls = ttk.Frame(self)
        self.controls.pack(fill="x", pady=(0, 10))
        self.note = tk.StringVar(self)
        ttk.Label(self, textvariable=self.note, style="Muted.TLabel").pack(fill="x", pady=(0, 6))

        panes = ttk.Panedwindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True)
        self.plot = SignalPlot(panes, on_seek, x_label)
        panes.add(self.plot, weight=5)

        table_frame = ttk.Frame(panes, width=390, padding=(6, 0, 0, 0))
        table_frame.grid_propagate(False)
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)
        self.table = ttk.Treeview(table_frame, show="headings", selectmode="browse")
        self.heading_font = font.Font(root=self, font=(FONT, 10, "bold"))
        vertical = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        horizontal = ttk.Scrollbar(table_frame, orient="horizontal", command=self.table.xview)
        self.table.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        panes.add(table_frame, weight=4)
        self.table.bind("<<TreeviewSelect>>", self.select_sample)
        self.table.tag_configure("even", background="#f1f6f9")

    def clear(self, description=""):
        self.note.set(description)
        self.plot.set_data(np.empty(0), (), "")
        self.fill_table(np.empty(0), ())

    def show(self, x_values, series, unit, index, *, columns=None, comparison=()):
        self.plot.set_data(x_values, series, unit, comparison=comparison)
        if columns is None:
            columns = tuple((f"{name} ({unit})", values) for name, values in series)
        self.fill_table(x_values, columns)
        self.set_index(index)

    def fill_table(self, x_values, columns):
        items = self.table.get_children()
        if items:
            self.table.delete(*items)
        keys = tuple(f"c{index}" for index in range(len(columns) + 1))
        self.table.configure(columns=keys)
        labels = (self.x_label, *(name for name, _ in columns))
        for key, label in zip(keys, labels):
            width = max(88, self.heading_font.measure(label) + 18)
            self.table.heading(key, text=label, anchor="e")
            self.table.column(key, width=width, minwidth=width, anchor="e", stretch=True)
        for index, row in enumerate(zip(x_values, *(values for _, values in columns))):
            tags = ("even",) if index % 2 == 0 else ()
            self.table.insert("", "end", iid=str(index), values=tuple(map(number, row)), tags=tags)

    def set_index(self, index):
        key = str(index)
        if not self.table.exists(key):
            return
        self.plot.set_index(index)
        if self.table.selection() != (key,):
            self.table.selection_set(key)
            self.table.focus(key)
        self.table.see(key)

    def select_sample(self, _event):
        selection = self.table.selection()
        if selection and int(selection[0]) != self.plot.index:
            self.on_seek(int(selection[0]))


def picker(parent, column, label, variable, values=(), *, width=16, editable=False, command=None):
    ttk.Label(parent, text=label).grid(row=0, column=column, sticky="w", padx=(0, 10))
    combo = ttk.Combobox(parent, textvariable=variable, values=values, width=width,
                         state="normal" if editable else "readonly")
    combo.grid(row=1, column=column, sticky="ew", padx=(0, 10), pady=(3, 0))
    if command is not None:
        combo.bind("<<ComboboxSelected>>", command)
    return combo


class DamperResultView(SignalView):
    def __init__(self, parent, on_seek):
        super().__init__(parent, on_seek)
        self.primary = tk.StringVar(self)
        self.secondary = tk.StringVar(self)
        self.field = tk.StringVar(self, value=next(iter(DAMPER_HISTORY_FIELDS)))
        self.member_names = ()
        self.histories = {}
        self.time = None
        self.primary_picker = picker(self.controls, 0, "避震器 A", self.primary, width=23, command=self.select_members)
        self.secondary_picker = picker(self.controls, 1, "避震器 B（選填）", self.secondary, width=23,
                                       command=self.select_members)
        picker(self.controls, 2, "曲線", self.field, tuple(DAMPER_HISTORY_FIELDS), command=self.display_result)

    def set_solution(self, model, result):
        self.member_names = tuple(member.name for member in model.deformable_members) if model else ()
        self.histories = damper_histories(model, result) if result is not None else {}
        self.time = result.time if result is not None else None
        if self.primary.get() not in self.member_names:
            self.primary.set(next(iter(self.member_names), ""))
        if self.secondary.get() not in self.member_names or self.secondary.get() == self.primary.get():
            self.secondary.set("")
        self.primary_picker.configure(state="readonly" if self.member_names else "disabled")
        self.secondary_picker.configure(state="readonly" if len(self.member_names) > 1 else "disabled")
        self.select_members()

    def select_members(self, _event=None):
        self.primary_picker.configure(values=tuple(name for name in self.member_names if name != self.secondary.get()))
        self.secondary_picker.configure(values=("", *(name for name in self.member_names if name != self.primary.get())))
        self.display_result()

    def display_result(self, _event=None):
        if not self.member_names:
            self.clear("請先從工具列的「變形元件…」新增避震器。")
            return
        if self.time is None:
            self.clear("設定「輪跳與轉向…」後按「開始求解」，即可查看目前工況的避震器行程。")
            return
        history_a = self.histories[self.primary.get()]
        history_b = self.histories.get(self.secondary.get())
        field = self.field.get()
        key, unit, note = DAMPER_HISTORY_FIELDS[field]
        primary = ((field, getattr(history_a, key) * 1000.),)
        comparison = ((field, getattr(history_b, key) * 1000.),) if history_b else ()
        columns = []
        for label, (quantity, units, _) in DAMPER_HISTORY_FIELDS.items():
            prefix = "A " if history_b else ""
            columns.append((f"{prefix}{label} ({units})", getattr(history_a, quantity) * 1000.))
            if history_b:
                columns.append((f"B {label} ({units})", getattr(history_b, quantity) * 1000.))
        if history_b:
            note += " A 為實線，B 為虛線。"
        self.note.set(note)
        self.show(self.time, primary, unit, self.plot.index or 0, columns=columns, comparison=comparison)


class DamperSweepView(SignalView):
    def __init__(self, parent, begin_job, set_status):
        super().__init__(parent, self.seek_sweep, "ROLL 幅度 (mm)")
        self.begin_job = begin_job
        self.set_status = set_status
        self.model = self.sweep = None
        self.sample_index = 0
        self.axles = {}
        self.member_names = ()
        self.busy = False

        self.axle = tk.StringVar(self)
        self.mode = tk.StringVar(self, value="ROLL")
        self.chart_field = tk.StringVar(self, value=DAMPER_SWEEP_FIELDS[0])
        self.primary = tk.StringVar(self)
        self.secondary = tk.StringVar(self)
        self.absolute = tk.BooleanVar(self)
        choices = ttk.Frame(self.controls)
        choices.pack(fill="x", pady=(0, 8))
        self.axle_picker = picker(choices, 0, "車軸", self.axle, width=9, command=self.invalidate)
        self.mode_picker = picker(choices, 1, "掃描", self.mode, ("ROLL", "HEAVE"),
                                  width=9, command=self.change_mode)
        self.primary_picker = picker(choices, 2, "避震器 A", self.primary, width=23, command=self.select_members)
        self.secondary_picker = picker(choices, 3, "避震器 B（選填）", self.secondary, width=23,
                                       command=self.select_members)
        self.chart_picker = picker(choices, 4, "曲線", self.chart_field, width=15, command=self.display_sweep)

        scan = ttk.Frame(self.controls)
        scan.pack(fill="x")
        self.inputs = {
            mode: tuple(tk.StringVar(self, value=value) for value in defaults)
            for mode, defaults in (("ROLL", ("0", "40", "0")), ("HEAVE", ("-40", "40", "0")))
        }
        for variables in self.inputs.values():
            for variable in variables:
                variable.trace_add("write", self.invalidate)
        self.fixed_label = tk.StringVar(self)
        self.entries = []
        for column, label in enumerate(("下限 (mm)", "上限 (mm)", "")):
            heading = ttk.Label(scan, text=label) if label else ttk.Label(scan, textvariable=self.fixed_label)
            heading.grid(row=0, column=column, sticky="w", padx=(0, 8))
            entry = ttk.Entry(scan, width=11)
            entry.grid(row=1, column=column, sticky="ew", padx=(0, 8))
            entry.bind("<Return>", lambda _event: self.run_sweep())
            self.entries.append(entry)
        self.abs_check = ttk.Checkbutton(scan, text="取絕對值（伸縮變化量）", variable=self.absolute,
                                         command=self.display_sweep)
        self.abs_check.grid(row=1, column=3, sticky="w", padx=(4, 12))
        self.run_button = ttk.Button(scan, text="計算掃描", command=self.run_sweep)
        self.run_button.grid(row=1, column=4, sticky="w")
        self.change_mode()

    def set_model(self, model):
        self.model = model
        labels = {"F": "前軸", "R": "後軸"}
        self.axles = {labels[axle]: axle for axle in damper_axles(model)}
        self.axle_picker.configure(values=tuple(self.axles))
        if self.axle.get() not in self.axles:
            self.axle.set(next(iter(self.axles), ""))
        self.member_names = tuple(member.name for member in model.deformable_members) if model else ()
        if self.primary.get() not in self.member_names:
            self.primary.set(next(iter(self.member_names), ""))
        if self.secondary.get() not in self.member_names or self.secondary.get() == self.primary.get():
            self.secondary.set("")
        self.invalidate()
        self.select_members()

    def set_busy(self, busy):
        self.busy = busy
        enabled = not busy and bool(self.axles and self.member_names)
        for combo in (self.axle_picker, self.mode_picker, self.primary_picker):
            combo.configure(state="readonly" if enabled else "disabled")
        self.secondary_picker.configure(state="readonly" if enabled and len(self.member_names) > 1 else "disabled")
        self.chart_picker.configure(state="readonly" if enabled and self.mode.get() == "ROLL" else "disabled")
        for widget in (*self.entries, self.abs_check, self.run_button):
            widget.configure(state="normal" if enabled else "disabled")

    def select_members(self, _event=None):
        self.primary_picker.configure(values=tuple(name for name in self.member_names if name != self.secondary.get()))
        self.secondary_picker.configure(values=("", *(name for name in self.member_names if name != self.primary.get())))
        self.display_sweep()

    def change_mode(self, _event=None):
        roll = self.mode.get() == "ROLL"
        self.x_label = self.plot.x_label = "ROLL 幅度 (mm)" if roll else "HEAVE (mm)"
        self.fixed_label.set("固定 HEAVE (mm)" if roll else "固定 ROLL (mm)")
        self.chart_picker.configure(values=DAMPER_SWEEP_FIELDS if roll else DAMPER_SWEEP_FIELDS[:1])
        if not roll:
            self.chart_field.set(DAMPER_SWEEP_FIELDS[0])
        for entry, variable in zip(self.entries, self.inputs[self.mode.get()]):
            entry.configure(textvariable=variable)
        self.invalidate()
        self.set_busy(self.busy)

    def invalidate(self, *_):
        self.sweep = None
        if not self.axles:
            note = "請載入具有同軸左右輪心 WC 的標準懸吊模型。"
        elif not self.member_names:
            note = "請先從工具列的「變形元件…」新增避震器。"
        else:
            note = (
                "ROLL 為左輪位移減右輪位移。左側 ROLL 取正，右側 ROLL 取負，兩者以相同幅度疊圖。\n"
                "HEAVE 為左右輪位移和。輪跳正值為上壓，其他車軸及齒條位移維持零。"
            )
        self.clear(note)

    def run_sweep(self):
        axle = self.axles.get(self.axle.get())
        if self.busy or axle is None or not self.member_names:
            return
        mode = self.mode.get()
        try:
            lower, upper, fixed = (float(variable.get()) for variable in self.inputs[mode])
        except ValueError:
            messagebox.showerror("避震器行程掃描失敗", "上下限與固定值須輸入數值。", parent=self.winfo_toplevel())
            return
        self.invalidate()
        self.begin_job(sweep_dampers, (self.model, axle, mode, (lower, upper), fixed), self.install_sweep,
                       f"正在計算{self.axle.get()}避震器 {mode} 行程…", "避震器行程掃描失敗")

    def install_sweep(self, sweep):
        self.sweep = sweep
        self.sample_index = int(np.argmin(np.abs(sweep.input_mm)))
        self.display_sweep()
        self.set_status(f"避震器行程掃描完成 · {len(sweep.input_mm)} 個 {self.mode.get()} 樣本")

    def display_sweep(self, _event=None):
        sweep = self.sweep
        if sweep is None:
            return
        roll = self.mode.get() == "ROLL"
        absolute = self.absolute.get()
        show_difference = roll and self.chart_field.get() == DAMPER_SWEEP_FIELDS[1]
        labels = ("左側 ROLL", "右側 ROLL") if roll else ("HEAVE",)
        measure = "伸縮變化量" if absolute else "軸向壓縮量"
        groups = []
        differences, columns = [], []
        for slot, name in (("A", self.primary.get()), ("B", self.secondary.get())):
            if not name:
                continue
            index = sweep.member_names.index(name)
            values = sweep.compression_mm[index]
            series = tuple(zip(labels, np.abs(values) if absolute else values))
            columns.extend((f"{slot} {label} {measure} (mm)", data) for label, data in series)
            if roll:
                difference = sweep.roll_difference_pct[index]
                differences.append((f"{slot} 左右差距 (%)", difference))
                if show_difference:
                    series = (("左右差距", difference),)
            groups.append(series)
        for label, (left, right) in zip(labels, sweep.wheel_travel_mm):
            columns.extend(((f"{label} 左輪 (mm)", left), (f"{label} 右輪 (mm)", right)))
        note = ("左側 ROLL 為正位移差，右側 ROLL 為負位移差。"
                if roll else "HEAVE 為左右輪位移和。")
        note += " 伸縮變化量為 |靜態長度 − 當前長度|。" if absolute else " 正值為壓縮，負值為伸長。"
        if self.secondary.get():
            note += " A 為實線，B 為虛線。"
        if roll:
            note += "\n左右差距為伸縮幅度差除以兩側平均幅度，兩側幅度皆接近零時顯示 0%。"
        self.note.set(note)
        unit = DAMPER_SWEEP_FIELDS[1] if show_difference else f"{measure} (mm)"
        self.show(sweep.input_mm, groups[0], unit, self.sample_index, columns=(*differences, *columns),
                  comparison=groups[1] if len(groups) > 1 else ())

    def seek_sweep(self, index):
        self.sample_index = index
        self.set_index(index)


class MotionSettings(tk.Toplevel):
    def __init__(self, owner, model, options, on_save):
        super().__init__(owner)
        self.transient(owner)
        self.title("輪跳與轉向")
        self.resizable(True, True)
        self.minsize(430, 240)
        self.model = model
        self.on_save = on_save
        self.duration = tk.StringVar(self, value=str(options.get("duration", model.duration)))
        self.samples = tk.StringVar(self, value=str(options.get("samples", model.samples)))
        self.amplitudes = {}
        self.profiles = {}
        self.heave_drives = []

        body = ttk.Frame(self, padding=15)
        body.pack(fill="both", expand=True)
        for row, label, variable in ((0, "總時間 (s)", self.duration), (1, "取樣數", self.samples)):
            ttk.Label(body, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(body, textvariable=variable, width=14).grid(row=row, column=1, sticky="e", pady=4)
        drives = ttk.Frame(body)
        drives.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(8, 8))
        body.columnconfigure(0, weight=1)
        body.rowconfigure(2, weight=1)
        canvas = tk.Canvas(drives, height=195, background=BACKGROUND, highlightthickness=0)
        scroll = ttk.Scrollbar(drives, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        rows = ttk.Frame(canvas)
        window = canvas.create_window(0, 0, window=rows, anchor="nw")
        rows.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        rows.columnconfigure(0, weight=1)
        for column, label in enumerate(("輪跳／轉向輸入", "行程形式", "振幅 (mm)")):
            ttk.Label(rows, text=label, style="Muted.TLabel").grid(row=0, column=column, sticky="w", padx=(0, 8))
        amplitudes = options.get("amplitudes_mm", {})
        profiles = options.get("profiles", {})
        editable = [drive for drive in model.drives if not drive.locked]
        for row, drive in enumerate(editable, 1):
            corner = drive.name.split("_", 1)[0]
            label = drive.name
            if drive.name == "FRONT_RACK":
                label = "轉向 · 前齒條"
            elif corner in CORNER_LABELS and drive.name in (f"{corner}_WC_Z", f"{corner}_UBJ_Z"):
                point = "輪心" if drive.name.endswith("_WC_Z") else "上球接點"
                label = f"{CORNER_LABELS[corner]}{point} Z ({corner})"
                self.heave_drives.append(drive.name)
            ttk.Label(rows, text=label, wraplength=210).grid(row=row, column=0, sticky="ew", pady=3)
            profile = tk.StringVar(self, value=PROFILE_LABELS[profiles.get(drive.name, drive.profile)])
            ttk.Combobox(rows, textvariable=profile, values=tuple(PROFILE_LABELS.values()), width=7,
                         state="readonly").grid(row=row, column=1, sticky="w", padx=(0, 12))
            value = tk.StringVar(self, value=str(amplitudes.get(drive.name, drive.amplitude_mm)))
            ttk.Entry(rows, textvariable=value, width=12).grid(row=row, column=2, sticky="e")
            self.amplitudes[drive.name] = value
            self.profiles[drive.name] = profile

        settings = model.reference_settings
        if settings is not None:
            presets = ttk.Frame(body)
            presets.grid(row=3, column=0, columnspan=2, sticky="w", pady=(0, 8))
            ttk.Label(presets, text=f"來源 · {settings.source}", style="Muted.TLabel").pack(anchor="w", pady=(0, 4))
            ttk.Button(presets, text=f"基準上壓 +{settings.bump_mm:g} mm",
                       command=lambda: self.set_reference_travel(settings.bump_mm)).pack(side="left", padx=(0, 8))
            ttk.Button(presets, text=f"基準下垂 −{settings.droop_mm:g} mm",
                       command=lambda: self.set_reference_travel(-settings.droop_mm)).pack(side="left")

        hint = ("Z 正值為上壓。轉向正值使前齒條沿軸朝車輛左側移動。"
                if "FRONT_RACK" in self.amplitudes else
                "Z 正值為上壓。匯入前輪 RACK 與 TOE 硬點可啟用齒條轉向。")
        ttk.Label(body, text=hint, wraplength=390, style="Muted.TLabel").grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        self.error = tk.StringVar(self)
        ttk.Label(body, textvariable=self.error, wraplength=390, foreground="#a3312d").grid(
            row=5, column=0, columnspan=2, sticky="w"
        )
        buttons = ttk.Frame(body)
        buttons.grid(row=6, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="套用", command=self.apply, style="Accent.TButton").pack(side="left")
        self.bind("<Return>", lambda _event: self.apply())
        self.bind("<Escape>", lambda _event: self.destroy())
        self.grab_set()

    def set_reference_travel(self, amplitude_mm):
        for name in self.heave_drives:
            self.amplitudes[name].set(f"{amplitude_mm:g}")
            self.profiles[name].set(PROFILE_LABELS["pulse"])

    def apply(self):
        try:
            duration = float(self.duration.get())
            samples = int(self.samples.get())
            amplitudes = {name: float(value.get()) for name, value in self.amplitudes.items()}
        except ValueError:
            self.error.set("總時間與振幅須為數字，取樣數須為整數。")
            return
        profile_keys = {label: key for key, label in PROFILE_LABELS.items()}
        profiles = {name: profile_keys[value.get()] for name, value in self.profiles.items()}
        options = {"duration": duration, "samples": samples, "amplitudes_mm": amplitudes, "profiles": profiles}
        try:
            make_motion(self.model, **options)
        except ValueError as error:
            self.error.set(str(error))
            return
        self.on_save(options)
        self.destroy()


class Workbench(ttk.Frame):
    def __init__(self, root, hardpoints=None, mechanism=None):
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        root.configure(background=BACKGROUND)
        root.option_add("*Font", f"{{{FONT}}} 10")
        style.configure(".", background=BACKGROUND, foreground=INK)
        style.configure("TButton", padding=(11, 6))
        style.configure("TNotebook.Tab", padding=(16, 7))
        style.configure("Treeview", rowheight=25, background="white", fieldbackground="white")
        style.configure("Treeview.Heading", background="#dfeaf0", font=(FONT, 10, "bold"))
        style.configure("Title.TLabel", font=(FONT, 21, "bold"))
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Warning.TButton", foreground="#9a4a00")
        style.configure("Accent.TButton", background="#006a63", foreground="white")
        style.map("Accent.TButton", background=[("disabled", "#e0e8ed"), ("active", "#00554e")],
                  foreground=[("disabled", MUTED)])
        super().__init__(root, padding=(18, 12))
        self.root = root
        root.title("Jacobian 運動學工作台")
        height = min(940, max(640, root.winfo_screenheight() - 80))
        root.geometry(f"1190x{height}")
        root.minsize(960, 640)
        self.pack(fill="both", expand=True)

        self.hardpoint_path = None
        self.project_source = None
        self.saved_project = None
        self.project_name = tk.StringVar(self, value="尚未匯入")
        self.status = tk.StringVar(self)
        self.model = self.result = self.ratio = None
        self.interferences = ()
        self.input_points = {}
        self.tires = {}
        self.options = {}
        self.job = None
        self.sample_index = 0
        self.geometry_selection_kind = None
        self.geometry_selections = {"point": (), "rod": ()}
        self.compare_geometry = tk.BooleanVar(self)
        self.play_anchor = self.play_job = None
        self.build_files()

        self.tabs = ttk.Notebook(self)
        self.build_viewport()
        self.build_signals()
        self.build_sweep()
        self.build_damper()
        self.build_ratio()
        self.tabs.bind("<<NotebookTabChanged>>", lambda _event: self.render_sample())

        footer = ttk.Frame(self)
        footer.pack(side="bottom", fill="x")
        ttk.Label(footer, textvariable=self.status, style="Muted.TLabel", wraplength=850).pack(
            side="left", fill="x", expand=True
        )
        self.progress = ttk.Progressbar(footer, mode="indeterminate", length=120)
        self.build_timeline()
        self.tabs.pack(fill="both", expand=True, pady=(12, 8))
        root.bind("<Control-o>", lambda _event: self.choose_hardpoints())
        root.bind("<Control-s>", lambda _event: self.save_current_project())
        root.protocol("WM_DELETE_WINDOW", self.close_project)
        self.set_paths(hardpoints or DEFAULT_HARDPOINTS, mechanism)

    def build_files(self):
        heading = ttk.Frame(self)
        heading.pack(fill="x")
        ttk.Label(heading, text="Jacobian 運動學工作台", style="Title.TLabel").pack(side="left")
        files = ttk.Frame(heading)
        files.pack(side="right")
        open_button = ttk.Button(files, text="匯入 CSV / JSON…", command=self.choose_hardpoints)
        open_button.pack(side="left")
        ttk.Label(files, textvariable=self.project_name, style="Muted.TLabel", width=32).pack(side="left", padx=8)
        self.save_button = ttk.Button(files, text="存檔", command=self.save_current_project, state="disabled")
        self.save_button.pack(side="left")
        self.task_buttons = [open_button]

        actions = ttk.Frame(self)
        actions.pack(fill="x", pady=(8, 0))
        geometry = ttk.Button(actions, text="硬點與輪胎設定…", command=self.configure_geometry)
        geometry.pack(side="left", padx=(0, 7))
        deformable = ttk.Button(actions, text="變形元件…", command=self.configure_deformable)
        deformable.pack(side="left", padx=(0, 7))
        settings = ttk.Button(actions, text="輪跳與轉向…", command=self.configure_motion)
        settings.pack(side="left")
        solve_button = ttk.Button(actions, text="開始求解", command=self.run_solve, style="Accent.TButton")
        solve_button.pack(side="left", padx=(7, 0))
        self.task_buttons.extend((geometry, deformable, settings, solve_button))
        self.motion_summary = tk.StringVar(self, value="使用機構預設的時間、樣本數與行程。")
        self.export_button = ttk.Button(actions, text="匯出 CSV…", command=self.save_results, state="disabled")
        self.export_button.pack(side="right")
        ttk.Label(actions, textvariable=self.motion_summary, style="Muted.TLabel").pack(side="left", padx=12)

    def build_viewport(self):
        self.geometry_tab = ttk.Frame(self.tabs, padding=(12, 10))
        self.tabs.add(self.geometry_tab, text="3D 運動")
        tools = ttk.Frame(self.geometry_tab)
        tools.pack(fill="x", pady=(0, 8))
        camera = tk.StringVar(self, value=next(iter(CAMERA_VIEWS)))
        views = ttk.Combobox(tools, textvariable=camera, values=tuple(CAMERA_VIEWS), width=10, state="readonly")
        views.pack(side="left", padx=(0, 6))
        views.bind("<<ComboboxSelected>>", lambda _event: self.viewport.set_view(CAMERA_VIEWS[camera.get()]))
        ttk.Button(tools, text="重設視角", command=lambda: self.viewport.reset_camera()).pack(side="left", padx=(0, 12))
        reference = tk.BooleanVar(self, value=True)
        ttk.Checkbutton(tools, text="靜態參考", variable=reference,
                        command=lambda: self.viewport.set_reference_visible(reference.get())).pack(side="left", padx=(0, 10))
        labels = tk.BooleanVar(self)
        ttk.Checkbutton(tools, text="節點名稱", variable=labels,
                        command=lambda: self.viewport.set_labels_visible(labels.get())).pack(side="left")
        self.comparison_checkbutton(tools).pack(side="left", padx=(10, 0))
        ttk.Label(tools, text="左鍵拖曳旋轉  ·  右鍵拖曳平移  ·  滾輪縮放", style="Muted.TLabel").pack(side="right")

        self.viewport = MechanismViewport(self.geometry_tab, on_pick=self.pick_geometry)
        self.viewport.set_reference_visible(reference.get())
        self.viewport.pack(fill="both", expand=True)

        inspect = ttk.Frame(self.geometry_tab)
        inspect.pack(fill="x", pady=(6, 0))
        selection = ttk.Frame(inspect)
        selection.pack(side="top", fill="x")
        self.geometry_names = tk.StringVar(self)
        ttk.Label(selection, textvariable=self.geometry_names, wraplength=740).pack(side="left")
        self.inspect_button = ttk.Button(selection, text="查看曲線", command=self.open_geometry_signal, state="disabled")
        self.inspect_button.pack(side="right", padx=(8, 0))
        self.geometry_detail = tk.StringVar(self)
        ttk.Label(inspect, textvariable=self.geometry_detail, style="Muted.TLabel", wraplength=740).pack(
            fill="x"
        )

    def build_timeline(self):
        bar = ttk.Frame(self)
        bar.pack(side="bottom", fill="x", pady=(0, 5))
        self.play_button = ttk.Button(bar, text="播放", width=6, command=self.toggle_playback, state="disabled")
        self.play_button.pack(side="left")
        self.reset_button = ttk.Button(bar, text="回到起點", command=lambda: self.seek_sample(0), state="disabled")
        self.reset_button.pack(side="left", padx=(6, 10))
        self.timeline_position = tk.DoubleVar(self)
        self.timeline = ttk.Scale(bar, variable=self.timeline_position, from_=0, to=1,
                                 command=lambda value: self.seek_sample(round(float(value))), state="disabled")
        self.timeline.pack(side="left", fill="x", expand=True)
        self.time_label = tk.StringVar(self, value="靜態姿態")
        ttk.Label(bar, textvariable=self.time_label, width=27, anchor="center").pack(side="left", padx=6)
        self.play_rate = tk.StringVar(self, value="1×")
        rate = ttk.Combobox(bar, textvariable=self.play_rate, values=tuple(PLAYBACK_RATES), width=5, state="readonly")
        rate.pack(side="left")
        rate.bind("<<ComboboxSelected>>", self.change_play_rate)
        self.loop_motion = tk.BooleanVar(self)
        ttk.Checkbutton(bar, text="循環", variable=self.loop_motion).pack(side="left", padx=(8, 0))
        self.interference_button = ttk.Button(
            bar, command=self.seek_first_interference, style="Warning.TButton",
        )

    def seek_sample(self, index):
        if self.result is not None:
            self.pause_playback()
            self.set_sample(index)

    def seek_first_interference(self):
        first = self.interferences[0]
        self.tabs.select(self.geometry_tab)
        self.seek_sample(first.first_sample)
        self.compare_geometry.set(True)
        self.set_geometry_selection("rod", (first.rod_a, first.rod_b))
        time = self.result.time[first.first_sample]
        self.status.set(f"桿件干涉 · {first.rod_a} 與 {first.rod_b} · {time:.6g} s")

    def set_sample(self, index):
        time = self.result.time
        self.sample_index = max(0, min(index, len(time) - 1))
        self.timeline_position.set(self.sample_index)
        self.time_label.set(f"{time[self.sample_index]:.5g} s   ·   {self.sample_index + 1} / {len(time)}")
        self.render_sample()

    def render_sample(self):
        if self.result is None:
            return
        current = self.tabs.select()
        if current == str(self.geometry_tab):
            self.viewport.set_frame(self.result.position[self.sample_index])
            self.show_geometry_detail()
        for view in (self.points, self.rods, self.pair):
            if current == str(view):
                view.set_index(self.sample_index)
                break
        if current == str(self.damper_tab) and self.damper_tab.select() == str(self.damper_result):
            self.damper_result.set_index(self.sample_index)

    def toggle_playback(self):
        if self.result is None:
            return
        if self.play_anchor is not None:
            self.pause_playback()
            return
        if self.sample_index == len(self.result.time) - 1:
            self.set_sample(0)
        self.play_anchor = (perf_counter(), self.result.time[self.sample_index])
        self.play_button.configure(text="暫停")
        self.play_job = self.after(33, self.play_tick)

    def play_tick(self):
        self.play_job = None
        clock, start = self.play_anchor
        time = self.result.time
        target = start + (perf_counter() - clock) * PLAYBACK_RATES[self.play_rate.get()]
        if target >= time[-1]:
            if not self.loop_motion.get():
                self.set_sample(len(time) - 1)
                self.pause_playback()
                return
            target = time[0] + (target - time[0]) % (time[-1] - time[0])
        index = closest_sample(time, target)
        if index != self.sample_index:
            self.set_sample(index)
        self.play_job = self.after(33, self.play_tick)

    def pause_playback(self):
        if self.play_job is not None:
            self.after_cancel(self.play_job)
            self.play_job = None
        self.play_anchor = None
        self.play_button.configure(text="播放")

    def change_play_rate(self, _event):
        if self.play_anchor is not None:
            self.play_anchor = (perf_counter(), self.result.time[self.sample_index])

    def pick_geometry(self, kind, name):
        selected = self.geometry_selections[kind]
        if not self.compare_geometry.get():
            selected = (name,)
        elif name in selected:
            selected = tuple(current for current in selected if current != name)
        else:
            selected = (*selected[-1:], name)
        self.set_geometry_selection(kind, selected)

    def pick_signal(self, kind, slot, name):
        selected = list(self.geometry_selections[kind])
        selected[slot:slot + 1] = (name,)
        self.set_geometry_selection(kind, tuple(selected))

    def set_geometry_selection(self, kind, selected):
        self.geometry_selections[kind] = selected
        self.geometry_selection_kind = kind
        self.sync_geometry_selection()
        (self.show_point if kind == "point" else self.show_rod)()

    def comparison_checkbutton(self, parent):
        return ttk.Checkbutton(parent, text="比較模式", variable=self.compare_geometry,
                               command=self.change_geometry_comparison)

    def change_geometry_comparison(self):
        if not self.compare_geometry.get():
            for kind, selected in self.geometry_selections.items():
                self.geometry_selections[kind] = selected[:1]
        self.sync_geometry_selection()
        self.show_point()
        self.show_rod()

    def sync_geometry_selection(self):
        points = self.geometry_selections["point"]
        rods = self.geometry_selections["rod"]
        self.viewport.set_selection(points, rods)
        point_names = "  /  ".join(f"{slot} {name}" for slot, name in zip("AB", points))
        rod_names = "  /  ".join(f"{slot} {name}" for slot, name in zip("AB", rods))
        self.geometry_names.set(f"節點  {point_names or '—'}    ｜    桿件  {rod_names or '—'}")
        choices = {
            "point": () if self.model is None else tuple(
                name for name in self.model.point_names if name not in self.model.internal_points
            ),
            "rod": () if self.model is None else tuple(link.name for link in self.model.links),
        }
        for kind, pickers in self.geometry_pickers.items():
            selected = self.geometry_selections[kind]
            primary, secondary = (selected + ("", ""))[:2]
            for slot, (value, combo) in enumerate(pickers):
                name, other = (primary, secondary) if slot == 0 else (secondary, primary)
                available = tuple(choice for choice in choices[kind] if choice != other)
                enabled = available and (slot == 0 or (primary and self.compare_geometry.get()))
                combo.configure(values=available, state="readonly" if enabled else "disabled")
                value.set(name)
        self.show_geometry_detail()

    def show_geometry_detail(self):
        self.inspect_button.configure(state="disabled")
        if self.model is None:
            self.geometry_detail.set("")
            return
        selected = self.geometry_selections.get(self.geometry_selection_kind, ())
        if not selected:
            self.geometry_detail.set("點選節點查看 XYZ 座標，或點選桿件查看兩端點與長度。")
            return
        position = self.model.reference if self.result is None else self.result.position[self.sample_index]
        kind = self.geometry_selection_kind
        details = [f"{slot} {self.describe_geometry(kind, name, position)}" for slot, name in zip("AB", selected)]
        self.geometry_detail.set("\n".join(details))
        if self.result is not None:
            self.inspect_button.configure(state="normal")

    def describe_geometry(self, kind, name, position):
        node_names = self.model.point_names
        if kind == "point":
            index = node_names.index(name)
            xyz = position[index] * 1000.
            values = "   ".join(f"{axis} {value:.3f}" for axis, value in zip(AXES, xyz))
            detail = f"{name}  ·  {values} mm"
            if self.result is not None:
                delta = self.result.displacement[self.sample_index, index] * 1000.
                changes = "   ".join(f"{axis} {value:.3f}" for axis, value in zip(DISPLACEMENT_AXES, delta))
                detail += f"  ·  {changes} mm"
            return detail
        link = next(link for link in self.model.links if link.name == name)
        a, b = node_names.index(link.a), node_names.index(link.b)
        length = np.linalg.norm(position[b] - position[a]) * 1000.
        detail = f"{name}  ·  {link.a} → {link.b}  ·  長度 {length:.3f} mm"
        if any(member.name == name for member in self.model.deformable_members):
            reference_length = np.linalg.norm(self.model.reference[b] - self.model.reference[a]) * 1000.
            detail += f"  ·  軸向壓縮量 {reference_length - length:.3f} mm"
        return detail

    def open_geometry_signal(self):
        if self.geometry_selection_kind is None or self.result is None:
            return
        self.tabs.select(self.points if self.geometry_selection_kind == "point" else self.rods)

    def build_signals(self):
        self.points = SignalView(self.tabs, self.seek_sample)
        self.rods = SignalView(self.tabs, self.seek_sample)
        self.tabs.add(self.points, text="節點")
        self.tabs.add(self.rods, text="桿件")

        self.point_field = tk.StringVar(self, value=next(iter(POINT_FIELDS)))
        self.rod_field = tk.StringVar(self, value=ROD_FIELDS[0])
        self.geometry_pickers = {
            "point": self.build_geometry_pickers(self.points.controls, "point", "節點"),
            "rod": self.build_geometry_pickers(self.rods.controls, "rod", "桿件"),
        }
        picker(self.points.controls, 2, "訊號", self.point_field, tuple(POINT_FIELDS), command=self.show_point)
        picker(self.rods.controls, 2, "訊號", self.rod_field, ROD_FIELDS, command=self.show_rod)

    def build_geometry_pickers(self, parent, kind, label):
        pickers = []
        for slot, suffix in enumerate("AB"):
            value = tk.StringVar(self)
            combo = picker(parent, slot, f"{label} {suffix}", value, width=25,
                           command=lambda _event, slot=slot, value=value: self.pick_signal(kind, slot, value.get()))
            pickers.append((value, combo))
        self.comparison_checkbutton(parent).grid(row=1, column=3, sticky="w", padx=(4, 0))
        return pickers

    def build_sweep(self):
        self.suspension_tab = ttk.Notebook(self.tabs)
        self.tabs.add(self.suspension_tab, text="懸吊特性")
        self.sweep_view = SignalView(self.suspension_tab, self.seek_sweep, "輪跳 (mm)")
        self.suspension_tab.add(self.sweep_view, text="單輪輪跳")
        self.sweep = None
        self.sweep_index = 0
        self.sweep_corner = tk.StringVar(self)
        self.sweep_field = tk.StringVar(self, value=next(iter(SWEEP_FIELDS)))
        controls = self.sweep_view.controls
        self.sweep_corner_picker = picker(controls, 0, "輪位", self.sweep_corner, width=11,
                                          command=self.change_sweep_corner)
        picker(controls, 1, "特性", self.sweep_field, tuple(SWEEP_FIELDS), width=19, command=self.show_sweep)
        self.sweep_limits, self.sweep_entries = self.sweep_inputs(
            controls, 2, (("輪跳下限 (mm)", "-30"), ("輪跳上限 (mm)", "20"),
                          ("齒條下限 (mm)", "-5"), ("齒條上限 (mm)", "5")),
            self.run_sweep, self.invalidate_sweep
        )
        self.sweep_button = ttk.Button(controls, text="計算掃描", command=self.run_sweep)
        self.sweep_button.grid(row=1, column=6, sticky="w")
        self.build_front_axle()

    def build_damper(self):
        self.damper_tab = ttk.Notebook(self.tabs)
        self.tabs.add(self.damper_tab, text="避震器行程")
        self.damper_result = DamperResultView(self.damper_tab, self.seek_sample)
        self.damper_tab.add(self.damper_result, text="求解結果")
        self.damper_view = DamperSweepView(self.damper_tab, self.begin_job, self.status.set)
        self.damper_tab.add(self.damper_view, text="ROLL／HEAVE 掃描")
        self.damper_tab.bind("<<NotebookTabChanged>>", lambda _event: self.render_sample())

    def sweep_inputs(self, controls, first_column, fields, run, invalidate):
        variables, entries = [], []
        for column, (label, default) in enumerate(fields, first_column):
            variable = tk.StringVar(self, value=default)
            ttk.Label(controls, text=label).grid(row=0, column=column, sticky="w", padx=(0, 8))
            entry = ttk.Entry(controls, textvariable=variable, width=11)
            entry.grid(row=1, column=column, padx=(0, 8), sticky="ew")
            entry.bind("<Return>", lambda _event: run())
            variable.trace_add("write", invalidate)
            variables.append(variable)
            entries.append(entry)
        return variables, entries

    def build_front_axle(self):
        self.axle_view = SignalView(self.suspension_tab, self.seek_axle, "齒條位移 (mm)")
        self.suspension_tab.add(self.axle_view, text="前軸轉向")
        self.axle_sweep = None
        self.axle_index = 0
        self.axle_field = tk.StringVar(self, value=next(iter(AXLE_FIELDS)))
        controls = self.axle_view.controls
        self.axle_inputs, self.axle_entries = self.sweep_inputs(
            controls, 0, (("Heave (mm)", "0"), ("Roll (°)", "0"), ("齒條下限 (mm)", "-10"),
                          ("齒條上限 (mm)", "10"), ("軸距 (mm，選填)", "")),
            self.run_axle_sweep, self.invalidate_axle_sweep
        )
        picker(controls, 5, "特性", self.axle_field, tuple(AXLE_FIELDS), width=17, command=self.show_axle_sweep)
        self.axle_button = ttk.Button(controls, text="計算掃描", command=self.run_axle_sweep)
        self.axle_button.grid(row=1, column=6, sticky="w")

    def update_sweep_model(self):
        corners = sweep_corners(self.model)
        self.sweep_choices = {f"{corner} {CORNER_LABELS[corner]}": corner for corner in corners}
        self.sweep_corner_picker.configure(values=tuple(self.sweep_choices))
        if self.sweep_corner.get() not in self.sweep_choices:
            self.sweep_corner.set(next(iter(self.sweep_choices), ""))
        self.damper_view.set_model(self.model)
        self.change_sweep_corner()
        self.invalidate_axle_sweep()

    def update_sweep_controls(self):
        corner = self.sweep_choices.get(self.sweep_corner.get())
        enabled = self.job is None and corner is not None
        steering = enabled and has_steering(self.model, corner)
        self.sweep_corner_picker.configure(state="readonly" if enabled else "disabled")
        for entry in self.sweep_entries[:2]:
            entry.configure(state="normal" if enabled else "disabled")
        for entry in self.sweep_entries[2:]:
            entry.configure(state="normal" if steering else "disabled")
        self.sweep_button.configure(state="normal" if enabled else "disabled")
        axle_state = "normal" if self.job is None and has_front_axle(self.model) else "disabled"
        for widget in (*self.axle_entries, self.axle_button):
            widget.configure(state=axle_state)
        self.damper_view.set_busy(self.job is not None)

    def change_sweep_corner(self, _event=None):
        self.update_sweep_controls()
        self.invalidate_sweep()

    def invalidate_sweep(self, *_):
        self.sweep = None
        corner = self.sweep_choices.get(self.sweep_corner.get())
        if corner is None:
            note = "請載入含輪心 WC 的標準懸吊硬點 CSV。"
        elif has_steering(self.model, corner):
            note = "輪跳正值為上壓。各曲線固定齒條在下限、中點或上限，其他輪位維持零輪跳。"
        else:
            note = "輪跳正值為上壓。齒條與其他輪位輪跳維持零。"
        self.sweep_view.clear(note)

    def run_sweep(self):
        if self.job is not None:
            return
        corner = self.sweep_choices.get(self.sweep_corner.get())
        if corner is None:
            return
        try:
            travel = tuple(float(variable.get()) for variable in self.sweep_limits[:2])
            rack = tuple(float(variable.get()) for variable in self.sweep_limits[2:]) if has_steering(
                self.model, corner
            ) else (0., 0.)
        except ValueError:
            messagebox.showerror("懸吊特性掃描失敗", "輪跳與齒條位移須輸入數值。", parent=self.root)
            return
        self.invalidate_sweep()
        self.begin_job(sweep_suspension, (self.model, corner, travel, rack, self.tires), self.install_sweep,
                       f"正在計算 {corner} 懸吊特性…", "懸吊特性掃描失敗")

    def install_sweep(self, sweep):
        self.sweep = sweep
        self.sweep_index = int(np.argmin(np.abs(sweep.travel_mm)))
        self.show_sweep()
        self.status.set(f"懸吊特性掃描完成 · {len(sweep.travel_mm)} 個輪跳樣本 × {len(sweep.rack_mm)} 個齒條位置")

    def show_sweep(self, _event=None):
        if self.sweep is None:
            return
        key, unit, note = SWEEP_FIELDS[self.sweep_field.get()]
        values = getattr(self.sweep, key)
        if key in ("track_mm", "wheelbase_mm") and not np.isfinite(values).any():
            note += " 需提供對應輪位的輪心 WC。"
        self.sweep_view.note.set(note)
        series = tuple((f"齒條 {rack:g} mm", row) for rack, row in zip(self.sweep.rack_mm, values))
        self.sweep_view.show(self.sweep.travel_mm, series, unit, self.sweep_index)

    def seek_sweep(self, index):
        self.sweep_index = index
        self.sweep_view.set_index(index)

    def invalidate_axle_sweep(self, *_):
        self.axle_sweep = None
        if has_front_axle(self.model):
            note = ("Heave 正值為前軸上壓，Roll 正值為左輪相對右輪上壓。\n"
                    "軸距留空時使用硬點的前後軸輪心。")
        else:
            note = "請載入具有 FL、FR 輪心 WC 與轉向拉桿 RACK、TOE 的標準懸吊 CSV。"
        self.axle_view.clear(note)

    def run_axle_sweep(self):
        if self.job is not None or not has_front_axle(self.model):
            return
        try:
            heave, roll, lower, upper = (float(value.get()) for value in self.axle_inputs[:4])
            wheelbase_text = self.axle_inputs[4].get().strip()
            wheelbase = float(wheelbase_text) if wheelbase_text else None
        except ValueError:
            messagebox.showerror("前軸掃描失敗", "Heave、Roll、齒條位移及軸距須輸入數值。", parent=self.root)
            return
        self.invalidate_axle_sweep()
        self.begin_job(sweep_front_axle, (self.model, heave, roll, (lower, upper), wheelbase, self.tires),
                       self.install_axle_sweep, "正在計算前軸轉向…", "前軸掃描失敗")

    def install_axle_sweep(self, sweep):
        self.axle_sweep = sweep
        self.axle_index = int(np.argmin(np.abs(sweep.rack_mm)))
        self.show_axle_sweep()
        self.status.set(f"前軸轉向掃描完成 · {len(sweep.rack_mm)} 個齒條位置")

    def show_axle_sweep(self, _event=None):
        sweep = self.axle_sweep
        if sweep is None:
            return
        field = self.axle_field.get()
        key, unit, note = AXLE_FIELDS[field]
        values = getattr(sweep, key)
        series = tuple(zip(("左前 FL", "右前 FR"), values)) if values.ndim == 2 else ((field, values),)
        left, right = sweep.travel_mm
        summary = f"前輪輪跳 FL {left:+.3g} mm ／ FR {right:+.3g} mm"
        if sweep.wheelbase_mm is None:
            summary += "   ·   填入軸距即可計算阿克曼率"
        else:
            summary += f"   ·   靜態前輪距 {sweep.track_mm:g} mm ／ 軸距 {sweep.wheelbase_mm:g} mm"
        self.axle_view.note.set(f"{summary}\n{note}")
        columns = (
            ("FL 轉角 (°)", sweep.steer_deg[0]), ("FR 轉角 (°)", sweep.steer_deg[1]),
            ("轉角差 (°)", sweep.difference_deg), ("阿克曼率 (%)", sweep.ackermann_pct),
            ("FL 航向 (°)", sweep.yaw_deg[0]), ("FR 航向 (°)", sweep.yaw_deg[1]),
        )
        self.axle_view.show(sweep.rack_mm, series, unit, self.axle_index, columns=columns)

    def seek_axle(self, index):
        self.axle_index = index
        self.axle_view.set_index(index)

    def build_ratio(self):
        self.pair = SignalView(self.tabs, self.seek_sample)
        self.tabs.add(self.pair, text="運動比與機械利益")
        self.input_name = tk.StringVar(self)
        self.output_name = tk.StringVar(self)
        self.input_direction = tk.StringVar(self, value="Z")
        self.output_direction = tk.StringVar(self, value="Z")
        self.rate_source = tk.StringVar(self, value=PATH_RATE)
        self.ratio_field = tk.StringVar(self, value=RATIO_FIELDS[0])
        self.drive_choices = {}

        axes = (*AXES, "-X", "-Y", "-Z")
        controls = self.pair.controls
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(2, weight=1)
        self.input_picker = picker(controls, 0, "輸入點", self.input_name, width=14, command=self.refresh_pair)
        self.output_picker = picker(controls, 2, "輸出點", self.output_name, width=14, command=self.refresh_pair)
        for column, label, variable in ((1, "輸入投影方向", self.input_direction),
                                        (3, "輸出投影方向", self.output_direction)):
            combo = picker(controls, column, label, variable, axes, width=9, editable=True, command=self.refresh_pair)
            combo.bind("<Return>", self.refresh_pair)
            variable.trace_add("write", self.invalidate_pair)
        self.rate_picker = picker(controls, 4, "比值基準", self.rate_source, (PATH_RATE,),
                                  width=16, command=self.refresh_pair)
        picker(controls, 5, "曲線", self.ratio_field, RATIO_FIELDS, width=12, command=self.show_pair)
        ttk.Button(controls, text="更新", command=self.refresh_pair).grid(row=1, column=6, sticky="se")

    def choose_hardpoints(self):
        if self.job is not None or self.root.grab_current() is not None:
            return
        path = filedialog.askopenfilename(parent=self.root, title="選擇懸吊專案",
                                         initialdir=str(self.hardpoint_path.parent) if self.hardpoint_path else None,
                                         filetypes=PROJECT_FILE_TYPES)
        if path and self.confirm_project_changes():
            self.set_paths(path)

    def project_state(self):
        return self.input_points, self.model.pushrod_mounts, self.tires, self.options, self.model.editable_deformable_members

    @property
    def project_changed(self):
        return self.model is not None and self.project_state() != self.saved_project

    def update_project_name(self):
        marker = " *" if self.project_changed else ""
        self.project_name.set(self.hardpoint_path.name + marker)

    def save_current_project(self):
        if self.model is None or self.root.grab_current() is not None:
            return False
        path = self.hardpoint_path
        json_project = isinstance(self.project_source, JsonProject)
        update_source = path != DEFAULT_HARDPOINTS.resolve() and (not json_project or self.project_source.native)
        if not update_source:
            extension = ".json" if json_project else ".csv"
            projects = EXAMPLES.parent / "projects"
            chosen = filedialog.asksaveasfilename(parent=self.root, title="另存懸吊專案",
                                                  initialdir=str(projects if projects.is_dir() else EXAMPLES.parent),
                                                  initialfile=f"suspension_project{extension}", defaultextension=extension,
                                                  filetypes=(("JSON project", "*.json"),) if json_project else CSV_FILE_TYPES)
            if not chosen:
                return False
            path = Path(chosen).resolve()
            if path in (self.hardpoint_path, DEFAULT_HARDPOINTS.resolve()):
                messagebox.showerror("請另存專案", "請選擇新的檔名來儲存專案。", parent=self.root)
                return False
        try:
            self.project_source = save_project(path, self.project_source, *self.project_state(),
                                                update_source=update_source)
        except (OSError, ValueError) as error:
            messagebox.showerror("存檔失敗", str(error), parent=self.root)
            return False
        self.hardpoint_path = path
        self.saved_project = self.project_state()
        self.update_project_name()
        self.status.set(f"已存檔 · {path.name}")
        return True

    def confirm_project_changes(self):
        if not self.project_changed:
            return True
        save = messagebox.askyesnocancel("儲存懸吊專案", f"「{self.hardpoint_path.name}」有尚未存檔的變更。要存檔嗎？",
                                         parent=self.root)
        if save is None:
            return False
        return not save or self.save_current_project()

    def close_project(self):
        dialog = self.root.grab_current()
        if dialog is not None:
            dialog.lift()
            return
        if self.confirm_project_changes():
            self.root.destroy()

    def set_paths(self, hardpoints, mechanism=None):
        try:
            path = Path(hardpoints).resolve()
            source, model, tires, options = load_project(path, mechanism)
        except (OSError, ValueError, RuntimeError) as error:
            self.status.set(str(error))
            return
        self.hardpoint_path, self.project_source = path, source
        self.mechanism = source.mechanism if isinstance(source, JsonProject) else mechanism
        self.model, self.input_points, self.tires, self.options = model, source.points, tires, options
        self.saved_project = self.project_state()
        self.update_project_name()
        self.save_button.configure(state="normal")
        self.motion_summary.set(f"{options['duration']:g} s · {options['samples']} 個樣本")
        self.clear_results(reset_selection=True)
        self.tabs.select(self.geometry_tab)
        if not model.standard_suspension:
            self.status.set(f"已載入自訂機構 · {model.name} · 按「開始求解」")
            return
        has_rack = any(drive.name == "FRONT_RACK" for drive in model.drives)
        rack = "已定義前齒條" if has_rack else "可補上 RACK 和 TOE 啟用前齒條"
        pushrods = sum(link.name.endswith("_PUSH_ROD") for link in model.links)
        self.status.set(f"已載入靜態幾何 · {rack} · {pushrods} 支推桿")
        if model.direct_pushrod_corners:
            corners = "、".join(model.direct_pushrod_corners)
            self.status.set(f"直推避震器配置 · {corners}")
            messagebox.showwarning("直推避震器配置",
                                   f"{corners} 未提供 ROCKER_A 與 ROCKER_B。PUSH_IN 已固定在車體。\n"
                                   "推桿已設為變形元件（Deformable member），可伸縮以維持懸吊運動自由度。"
                                   "目前使用直推避震器懸吊設計。", parent=self.root)

    def read_model(self):
        if self.model is None:
            raise ValueError("請選擇有效的 CSV 或 JSON 懸吊專案。")
        return self.model

    def configure_deformable(self):
        try:
            model = self.read_model()
        except ValueError as error:
            messagebox.showerror("無法讀取機構", str(error), parent=self.root)
            return
        self.pause_playback()
        DeformableSettings(self.root, model, self.set_deformable_members)

    def set_deformable_members(self, members):
        if members == self.model.editable_deformable_members:
            return
        self.model = load_model(self.hardpoint_path, self.mechanism, points=self.input_points,
                                pushrod_mounts=dict(self.model.pushrod_mounts), deformable_members=members)
        self.update_project_name()
        self.clear_results(reset_selection=True)
        self.tabs.select(self.geometry_tab)
        self.status.set("變形元件已更新。按「開始求解」計算新的曲線。")

    def configure_geometry(self):
        try:
            model = self.read_model()
        except (OSError, ValueError) as error:
            messagebox.showerror("無法讀取機構", str(error), parent=self.root)
            return
        tires = {corner: self.tires.get(corner, TireSetup()) for corner in CORNER_LABELS
                 if f"{corner}_WC" in model.point_names}
        self.pause_playback()
        GeometrySettings(self.root, self.input_points, tires, model.pushrod_mounts, self.set_geometry)

    def set_geometry(self, points, tires, pushrod_mounts):
        coordinates_changed = points != self.input_points
        mounts_changed = pushrod_mounts != dict(self.model.pushrod_mounts)
        geometry_changed = coordinates_changed or mounts_changed
        alignment_changed = any((setup.toe_deg, setup.camber_deg) != (previous.toe_deg, previous.camber_deg)
                                for corner, setup in tires.items()
                                for previous in (self.tires.get(corner, TireSetup()),))
        if geometry_changed:
            model = load_model(self.hardpoint_path, self.mechanism, points=points,
                               pushrod_mounts=pushrod_mounts, deformable_members=self.model.editable_deformable_members)
            self.model, self.input_points = model, points
        self.tires = tires
        self.update_project_name()
        if geometry_changed:
            self.clear_results(reset_selection=True)
            updated = "推桿外端接法" if mounts_changed else "硬點"
            self.status.set(f"{updated}已更新。按「開始求解」計算新的曲線。")
        else:
            positions = None if self.result is None else self.result.position
            self.viewport.set_model(self.model, positions, tires=tires)
            if positions is not None:
                self.viewport.set_frame(positions[self.sample_index])
            if alignment_changed:
                self.invalidate_sweep()
                self.invalidate_axle_sweep()
                self.status.set("靜態 Toe 與 Camber 已套用至 3D 視圖。可重新計算懸吊特性。")
            else:
                self.status.set("輪胎尺寸已套用至 3D 視圖。")
        self.tabs.select(self.geometry_tab)

    def configure_motion(self):
        try:
            model = self.read_model()
        except (OSError, ValueError) as error:
            messagebox.showerror("無法讀取機構", str(error), parent=self.root)
            return
        MotionSettings(self.root, model, self.options, self.set_motion)

    def set_motion(self, options):
        self.options = options
        self.update_project_name()
        self.motion_summary.set(f"{options['duration']:g} s · {options['samples']} 個樣本")
        self.clear_results()
        self.status.set("輪跳與轉向設定已更新。按「開始求解」計算新的曲線。")

    def clear_results(self, *, reset_selection=False):
        self.pause_playback()
        self.result = self.ratio = None
        self.interferences = ()
        self.interference_button.pack_forget()
        self.sample_index = 0
        self.timeline_position.set(0)
        self.timeline.configure(state="disabled", to=1)
        self.play_button.configure(state="disabled")
        self.reset_button.configure(state="disabled")
        self.time_label.set("靜態姿態")
        self.export_button.configure(state="disabled")
        self.points.clear("完成求解後可檢視曲線與樣本。")
        self.rods.clear("完成求解後可檢視曲線與樣本。")
        self.damper_result.set_solution(self.model, None)
        self.pair.clear()
        for combo in (self.input_picker, self.output_picker):
            combo.configure(values=())
        self.rate_picker.configure(values=(PATH_RATE,))
        if reset_selection:
            self.geometry_selection_kind = None
            self.geometry_selections = {"point": (), "rod": ()}
            for variable in (self.input_name, self.output_name, self.rate_source):
                variable.set("")
            self.update_sweep_model()
        self.viewport.set_model(self.model, tires=self.tires)
        self.sync_geometry_selection()

    def run_solve(self):
        if self.job is not None:
            return
        try:
            model = self.read_model()
            motion = make_motion(model, **self.options)
        except (OSError, ValueError) as error:
            messagebox.showerror("無法開始求解", str(error), parent=self.root)
            return
        self.clear_results()
        self.begin_job(_solve_with_interferences, (model, motion), self.install_result,
                       f"正在求解 {model.name} 並檢查桿件干涉…", "求解失敗")

    def begin_job(self, operation, args, install, working, error_title):
        self.job = SimpleQueue()
        for button in self.task_buttons:
            button.configure(state="disabled")
        self.update_sweep_controls()
        self.progress.pack(side="right")
        self.progress.start(12)
        self.status.set(working)
        Thread(target=self.compute_job, args=(self.job, operation, args), daemon=True).start()
        self.after(50, self.finish_job, install, error_title)

    @staticmethod
    def compute_job(queue, operation, args):
        try:
            queue.put((operation(*args), None))
        except Exception as error:
            queue.put((None, error))

    def finish_job(self, install, error_title):
        try:
            result, error = self.job.get_nowait()
        except Empty:
            self.after(50, self.finish_job, install, error_title)
            return
        self.job = None
        self.progress.stop()
        self.progress.pack_forget()
        for button in self.task_buttons:
            button.configure(state="normal")
        self.update_sweep_controls()
        if error is not None:
            self.status.set(error_title)
            messagebox.showerror(error_title, str(error), parent=self.root)
            return
        install(result)

    def install_result(self, solution):
        result, self.interferences = solution
        self.result = result
        self.viewport.set_model(self.model, result.position, tires=self.tires)
        self.timeline.configure(state="normal", to=len(result.time) - 1)
        self.play_button.configure(state="normal")
        self.reset_button.configure(state="normal")
        names = tuple(name for name in self.model.point_names if name not in self.model.internal_points)
        rods = tuple(link.name for link in self.model.links)
        for combo in (self.input_picker, self.output_picker):
            combo.configure(values=names)
        drives = tuple(drive for drive in self.model.drives if not drive.locked)
        self.drive_choices = {f"驅動：{drive.name}": drive.name for drive in drives}
        sources = (PATH_RATE, *self.drive_choices)
        self.rate_picker.configure(values=sources)
        if self.rate_source.get() not in sources:
            self.rate_source.set(next(iter(self.drive_choices), PATH_RATE))
        if not self.compare_geometry.get():
            first = drives[0].point if drives and drives[0].point in names else names[0]
            if not self.geometry_selections["point"]:
                self.geometry_selections["point"] = (first,)
            if not self.geometry_selections["rod"]:
                self.geometry_selections["rod"] = rods[:1]
        if self.input_name.get() not in names:
            self.set_default_pair(result, drives)
        self.show_point()
        self.show_rod()
        self.damper_result.set_solution(self.model, result)
        self.refresh_pair()
        self.sync_geometry_selection()
        self.set_sample(0)
        self.export_button.configure(state="normal")
        summary = f"{self.model.name} · {len(names)} 個節點 · {len(rods)} 根桿件 · {len(result.time)} 個樣本"
        if self.interferences:
            count = len(self.interferences)
            summary += f" · {count} 組桿件干涉"
            self.interference_button.configure(text=f"首處干涉（{count} 組）")
            self.interference_button.pack(side="left", padx=(8, 0))
        self.status.set(summary)
        if self.interferences:
            self.warn_interferences()

    def warn_interferences(self):
        limit = 6
        names = [
            f"{record.rod_a} 與 {record.rod_b} · 首次 {self.result.time[record.first_sample]:.6g} s"
            for record in self.interferences[:limit]
        ]
        remaining = len(self.interferences) - len(names)
        if remaining:
            names.append(f"另有 {remaining} 組。")
        tolerance = INTERFERENCE_TOLERANCE_M * 1000.
        message = (f"求解完成。{len(self.interferences)} 組桿件的中心線在取樣姿態中相交"
                   f"或距離在 {tolerance:g} mm 以內。\n\n"
                   + "\n".join(names)
                   + "\n\n可點「首處干涉」前往該時間的 3D 姿態。")
        messagebox.showwarning("桿件干涉警告", message, parent=self.root)

    def set_default_pair(self, result, drives):
        names = self.model.point_names
        first = drives[0].point if drives else names[0]
        direction = np.asarray(drives[0].direction if drives else (0, 0, 1))
        axis = next((label for vector, label in zip(np.eye(3), AXES) if np.allclose(vector, direction)), None)
        direction_text = axis or ", ".join(str(component) for component in direction)
        output = ""
        if len(names) > 1:
            source = result.sensitivity[..., result.drive_names.index(drives[0].name)] if drives else result.velocity
            projected = np.abs(source @ direction)
            motion = np.max(np.where(np.isfinite(projected), projected, 0), axis=0)
            motion[names.index(first)] = -1
            for name in self.model.internal_points:
                motion[names.index(name)] = -np.inf
            output = "FL_UBJ" if first == "FL_WC" and "FL_UBJ" in names else names[int(np.argmax(motion))]
        self.input_direction.set(direction_text)
        self.output_direction.set(direction_text)
        self.input_name.set(first)
        self.output_name.set(output)

    def show_point(self, _event=None):
        if self.result is None:
            return
        selected = self.geometry_selections["point"]
        if not selected:
            self.points.clear("請選擇節點以查看曲線。")
            return
        field, unit, magnitude_label = POINT_FIELDS[self.point_field.get()]
        axes = AXES
        note = "模型座標系中的 X、Y、Z 分量。"
        if field == "displacement":
            axes = DISPLACEMENT_AXES
            note = "各軸位移為當前位置減去該節點的原始靜態座標。"
        elif field == "position":
            note = "模型座標系中的絕對 X、Y、Z 位置。"
        if magnitude_label:
            note += f" {magnitude_label}為三軸分量的向量大小。"
        vectors = getattr(self.result, field)
        groups = []
        for name in selected:
            index = self.result.point_names.index(name)
            components = vectors[:, index, :] * 1000
            series = tuple(zip(axes, components.T))
            if magnitude_label:
                series += ((magnitude_label, np.linalg.norm(components, axis=1)),)
            groups.append(series)
        self.show_geometry_series(self.points, groups, unit, note)

    def show_rod(self, _event=None):
        if self.result is None:
            return
        if not self.model.links:
            self.rods.clear("本機構沒有定義桿件。")
            return
        selected = self.geometry_selections["rod"]
        if not selected:
            self.rods.clear("請選擇桿件以查看曲線。")
            return
        indices = [self.result.link_names.index(name) for name in selected]
        field = self.rod_field.get()
        if field in ("長度", "軸向壓縮量"):
            groups = []
            for index in indices:
                link = self.model.links[index]
                a, b = (self.result.point_names.index(name) for name in (link.a, link.b))
                delta = self.result.position[:, b] - self.result.position[:, a]
                length = np.linalg.norm(delta, axis=1)
                static_length = np.linalg.norm(self.model.reference[b] - self.model.reference[a])
                values = length if field == "長度" else static_length - length
                groups.append(((field, values * 1000.),))
            note = ("兩端點的直線距離。" if field == "長度" else
                    "靜態長度減去當前長度。正值表示壓縮，負值表示伸長。")
            self.show_geometry_series(self.rods, groups, "mm", note)
            return
        if field == ROD_FIELDS[0]:
            fields = (
                ("方位角", "azimuth_deg"), ("仰角", "elevation_deg"), ("與初始桿軸夾角", "angle_deg")
            )
            groups = [tuple((label, getattr(self.result, key)[:, index]) for label, key in fields)
                      for index in indices]
            unit = "deg"
        else:
            key, unit = ("angular_velocity", "rad/s") if field == ROD_FIELDS[1] else (
                "angular_acceleration", "rad/s²"
            )
            values = getattr(self.result, key)
            groups = [tuple(zip(AXES, values[:, index, :].T)) for index in indices]
        note = ("輪向方位角從車輛 +X 朝 +Y 為正，可用於查看轉向及輪跳造成的前束變化。"
                if any(name.endswith("_WHEEL_HEADING") for name in selected) else
                "曲線表示桿軸方向的變化。繞桿軸的自轉需額外的幾何定義。")
        self.show_geometry_series(self.rods, groups, unit, note)

    def show_geometry_series(self, view, groups, unit, note):
        primary, *others = groups
        secondary = others[0] if others else ()
        columns = None
        if secondary:
            note += " A 為實線，B 為虛線。"
            columns = tuple(
                (f"{slot} {label} ({unit})", values)
                for pair in zip(primary, secondary)
                for slot, (label, values) in zip("AB", pair)
            )
        elif self.compare_geometry.get():
            note += " 選擇 B 以比較兩筆資料。"
        view.note.set(note)
        view.show(self.result.time, primary, unit, self.sample_index, columns=columns, comparison=secondary)

    def invalidate_pair(self, *_):
        if self.ratio is not None:
            self.ratio = None
            self.pair.clear("方向已變更。按「更新」或 Enter 重新計算。")

    def refresh_pair(self, _event=None):
        if self.result is None:
            return
        self.ratio = None
        if not self.input_name.get() or not self.output_name.get():
            self.pair.clear("選擇兩個不同的點以比較投影運動。")
            return
        if self.input_name.get() == self.output_name.get():
            self.pair.clear("輸入點與輸出點請選擇不同節點。")
            return
        try:
            self.ratio = motion_ratio(self.result, self.input_name.get(), self.output_name.get(),
                                      read_direction(self.input_direction.get()),
                                      read_direction(self.output_direction.get()),
                                      drive=self.drive_choices.get(self.rate_source.get()))
        except ValueError as error:
            self.pair.clear(str(error))
            return
        self.show_pair()

    def show_pair(self, _event=None):
        ratio = self.ratio
        if ratio is None:
            return
        field = self.ratio_field.get()
        values = ratio.motion_ratio if field == RATIO_FIELDS[0] else ratio.mechanical_advantage
        path = ratio.drive is None
        rate_unit, scale = ("mm/s", 1000) if path else ("mm/mm", 1)
        columns = (
            (RATIO_FIELDS[0], ratio.motion_ratio),
            (RATIO_FIELDS[1], ratio.mechanical_advantage),
            ("輸入位移 (mm)", ratio.input_displacement * 1000),
            ("輸出位移 (mm)", ratio.output_displacement * 1000),
            (f"輸入變化率 ({rate_unit})", ratio.input_rate * scale),
            (f"輸出變化率 ({rate_unit})", ratio.output_rate * scale),
        )
        self.pair.note.set("運動比 = 輸出／輸入。理想機械利益 = |輸入／輸出|。無定義的樣本以「—」顯示。")
        self.pair.show(ratio.time_s, ((field, values),), "無因次", self.sample_index, columns=columns)

    def save_results(self):
        if self.result is None:
            return
        self.refresh_pair()
        directory = filedialog.askdirectory(parent=self.root, title="選擇 CSV 匯出資料夾", mustexist=True)
        if not directory:
            return
        target = Path(directory)
        names = ("point_kinematics.csv", "rod_kinematics.csv")
        if self.ratio is not None:
            names += ("motion_ratio.csv",)
        existing = [name for name in names if (target / name).exists()]
        if existing and not messagebox.askyesno("覆寫 CSV 檔案", f"覆寫這些檔案？\n\n" + "\n".join(existing),
                                                parent=self.root):
            return
        try:
            files = export_results(target, self.result, self.ratio)
        except (OSError, ValueError) as error:
            messagebox.showerror("匯出失敗", str(error), parent=self.root)
            return
        self.status.set(f"已匯出 {len(files)} 張表格至 {target}")


def launch(hardpoints=None, mechanism=None):
    root = tk.Tk()
    Workbench(root, hardpoints, mechanism)
    root.mainloop()

