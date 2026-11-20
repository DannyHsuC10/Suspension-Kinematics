"""Edit hardpoints, suspension mounts, tires, and deformable members."""

from math import isfinite
import tkinter as tk
from tkinter import ttk

from .model import CHASSIS, CORNERS, DeformableEnd, DeformableMember
from .wheel_frame import TireSetup


class GeometrySettings(tk.Toplevel):
    def __init__(self, parent, points, tires, pushrod_mounts, on_apply):
        super().__init__(parent)
        self.transient(parent)
        self.title("硬點與輪胎設定")
        self.geometry("700x720" if pushrod_mounts else "700x660")
        self.minsize(620, 600 if pushrod_mounts else 500)
        self.points = points
        self.original = {name: tuple(f"{coordinate * 1000.:.12g}" for coordinate in xyz)
                         for name, xyz in points.items()}
        self.draft = self.original.copy()
        self.on_apply = on_apply
        self.active_point = None
        self.tire_fields = {}
        self.pushrod_fields = {}

        body = ttk.Frame(self, padding=14)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)
        ttk.Label(body, text="點選硬點列，在表格下方編輯車輛 XYZ 座標（mm）。").grid(
            row=0, column=0, sticky="w", pady=(0, 5))
        table = ttk.Frame(body)
        table.grid(row=1, column=0, sticky="nsew")
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)
        self.tree = ttk.Treeview(table, columns=("name", "x", "y", "z"), show="headings", selectmode="browse")
        for key, title in zip(("name", "x", "y", "z"), ("硬點", "X (mm)", "Y (mm)", "Z (mm)")):
            anchor = "w" if key == "name" else "e"
            self.tree.heading(key, text=title, anchor=anchor)
            self.tree.column(key, width=160 if key == "name" else 125, minwidth=50, anchor=anchor)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=scroll.set)
        self.keys = {str(i): name for i, name in enumerate(points)}
        for key, name in self.keys.items():
            self.tree.insert("", "end", iid=key, values=(name, *(f"{v * 1000.:.9g}" for v in points[name])))

        edit = ttk.Frame(body)
        edit.grid(row=2, column=0, sticky="ew", pady=(8, 12))
        self.edit_label = tk.StringVar(self)
        ttk.Label(edit, textvariable=self.edit_label, width=20).pack(side="left")
        self.coordinates = []
        for axis in ("X", "Y", "Z"):
            ttk.Label(edit, text=axis).pack(side="left", padx=(0, 3))
            value = tk.StringVar(self)
            ttk.Entry(edit, textvariable=value, width=9).pack(side="left", fill="x", expand=True, padx=(0, 8))
            self.coordinates.append(value)
        self.tree.bind("<<TreeviewSelect>>", self.select_point)
        first = next(iter(self.keys))
        self.tree.selection_set(first)
        self.select_point()

        if pushrod_mounts:
            pushrod_frame = ttk.LabelFrame(body, text="推桿外端所屬剛體", padding=8)
            pushrod_frame.grid(row=3, column=0, sticky="ew", pady=(0, 8))
            for row, (corner, mount) in enumerate(pushrod_mounts):
                ttk.Label(pushrod_frame, text=corner).grid(row=row, column=0, sticky="w", padx=(0, 12))
                value = self.pushrod_fields[corner] = tk.StringVar(self, value=mount)
                for column, (key, label) in enumerate(
                        (("UCA", "上 A 臂"), ("LCA", "下 A 臂"), ("UPRIGHT", "羊角 (Upright)")), 1):
                    ttk.Radiobutton(pushrod_frame, text=label, variable=value, value=key).grid(
                        row=row, column=column, sticky="w", padx=(0, 14))

        tire_frame = ttk.LabelFrame(body, text="輪胎尺寸與靜態定位", padding=8)
        tire_frame.grid(row=4, column=0, sticky="ew")
        for column, label in enumerate(("輪位", "寬度 (mm)", "半徑 (mm)", "Toe (°)", "Camber (°)")):
            ttk.Label(tire_frame, text=label).grid(row=0, column=column, padx=(0, 10), sticky="w")
        if not tires:
            ttk.Label(tire_frame, text="匯入輪心（WC）後可設定輪胎。").grid(row=1, column=0, columnspan=5)
        for row, (corner, setup) in enumerate(tires.items(), 1):
            ttk.Label(tire_frame, text=corner).grid(row=row, column=0, sticky="w")
            values = tuple(tk.StringVar(self, value="" if value is None else str(value))
                           for value in (setup.width_mm, setup.radius_mm, setup.toe_deg, setup.camber_deg))
            for column, value in enumerate(values, 1):
                ttk.Entry(tire_frame, textvariable=value, width=11).grid(row=row, column=column, padx=(0, 10), pady=2)
            self.tire_fields[corner] = values
        if tires:
            ttk.Label(tire_frame, text="Toe 正值為內束，留空採 FORWARD。Camber 正值為輪胎頂端朝外。",
                      wraplength=560).grid(row=len(tires) + 1, column=0, columnspan=5, sticky="w", pady=(4, 0))
        self.error = tk.StringVar(self)
        ttk.Label(body, textvariable=self.error, foreground="#a3312d", wraplength=510).grid(
            row=5, column=0, sticky="w", pady=(6, 0))
        actions = ttk.Frame(body)
        actions.grid(row=6, column=0, sticky="e", pady=(8, 0))
        ttk.Button(actions, text="取消", command=self.destroy).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="套用", command=self.apply, style="Accent.TButton").pack(side="left")
        self.bind("<Escape>", lambda _event: self.destroy())
        self.grab_set()

    def store_point(self):
        if self.active_point is None:
            return
        key = self.active_point
        name = self.keys[key]
        values = tuple(entry.get() for entry in self.coordinates)
        if values != self.draft[name]:
            self.draft[name] = values
            self.tree.item(key, values=(name, *values))

    def select_point(self, _event=None):
        selection = self.tree.selection()
        if not selection or selection[0] == self.active_point:
            return
        self.store_point()
        self.active_point = selection[0]
        name = self.keys[self.active_point]
        self.edit_label.set(name)
        for entry, value in zip(self.coordinates, self.draft[name]):
            entry.set(value)

    @staticmethod
    def value(text, label, *, positive=False):
        try:
            value = float(text)
            if isfinite(value) and (not positive or value > 0.):
                return value
        except (ValueError, OverflowError):
            pass
        requirement = "大於零的有限數值" if positive else "有限數值"
        raise ValueError(f"{label} 需要{requirement}。")

    def apply(self):
        self.store_point()
        try:
            points = {name: tuple(old if text == original else self.value(text, f"{name} {axis}") / 1000.
                                  for axis, text, original, old in zip("XYZ", values, self.original[name], self.points[name]))
                      for name, values in self.draft.items()}
            tires = {}
            for corner, fields in self.tire_fields.items():
                width, radius, toe, camber = (field.get().strip() for field in fields)
                toe_deg = self.value(toe, f"{corner} Toe") if toe else None
                camber_deg = self.value(camber, f"{corner} Camber")
                if abs(toe_deg or 0.) >= 90. or abs(camber_deg) >= 90.:
                    raise ValueError(f"{corner} 的 Toe 和 Camber 須介於 −90° 與 90° 之間。")
                tires[corner] = TireSetup(self.value(width, f"{corner} 寬度", positive=True),
                                          self.value(radius, f"{corner} 半徑", positive=True), toe_deg, camber_deg)
            pushrod_mounts = {corner: field.get() for corner, field in self.pushrod_fields.items()}
            self.on_apply(points, tires, pushrod_mounts)
        except (OSError, ValueError) as error:
            self.error.set(str(error))
            return
        self.destroy()


class DeformableSettings(tk.Toplevel):
    def __init__(self, parent, model, on_apply):
        super().__init__(parent)
        self.transient(parent)
        self.title("變形元件")
        self.resizable(False, False)
        self.members = model.deformable_members
        self.on_apply = on_apply
        used_names = {link.name.casefold() for link in model.links}
        self.default_name = next(f"DAMPER_{i}" for i in range(1, len(used_names) + 2)
                                 if f"damper_{i}" not in used_names)

        body = ttk.Frame(self, padding=14)
        body.pack(fill="both", expand=True)
        choices = ttk.Frame(body)
        choices.pack(fill="x", pady=(0, 10))
        ttk.Label(choices, text="元件").pack(side="left", padx=(0, 8))
        self.member_picker = ttk.Combobox(choices, state="readonly",
                                          values=("新增變形元件", *(member.name for member in self.members)))
        self.member_picker.pack(side="left", fill="x", expand=True)
        self.member_picker.bind("<<ComboboxSelected>>", self.select_member)
        self.member_picker.current(0)

        name_row = ttk.Frame(body)
        name_row.pack(fill="x", pady=(0, 8))
        ttk.Label(name_row, text="名稱").pack(side="left", padx=(0, 8))
        self.name = tk.StringVar(self)
        name_entry = ttk.Entry(name_row, textvariable=self.name)
        name_entry.pack(side="left", fill="x", expand=True)
        ttk.Label(body, text="輸入兩端的靜態車輛座標，並各自選擇固定位置。").pack(anchor="w")

        rockers = dict(model.pushrod_mounts)
        self.ends = []
        for label in "AB":
            frame = ttk.LabelFrame(body, text=f"端點 {label}", padding=8)
            frame.pack(fill="x", pady=(8, 0))
            coordinates = []
            for column, axis in enumerate("XYZ"):
                ttk.Label(frame, text=f"{axis} (mm)").grid(row=0, column=column, sticky="w")
                value = tk.StringVar(self)
                ttk.Entry(frame, textvariable=value, width=13).grid(
                    row=1, column=column, sticky="ew", padx=(0, 8), pady=(2, 5))
                coordinates.append(value)
            mounts = ttk.Frame(frame)
            mounts.grid(row=2, column=0, columnspan=3, sticky="w")
            mount = tk.StringVar(self, value=CHASSIS)
            ttk.Radiobutton(mounts, text="車體", variable=mount, value=CHASSIS).pack(side="left", padx=(0, 6))
            for corner in CORNERS:
                radio = ttk.Radiobutton(mounts, text=f"{corner} 搖臂", variable=mount, value=corner)
                radio.pack(side="left", padx=(0, 6))
                if corner not in rockers:
                    radio.state(["disabled"])
            self.ends.append((coordinates, mount))

        self.error = tk.StringVar(self)
        ttk.Label(body, textvariable=self.error, foreground="#a3312d", wraplength=480).pack(
            anchor="w", pady=(8, 0))
        actions = ttk.Frame(body)
        actions.pack(fill="x", pady=(8, 0))
        self.remove_button = ttk.Button(actions, text="移除元件", command=self.remove)
        self.remove_button.pack(side="left")
        ttk.Button(actions, text="套用", command=self.apply, style="Accent.TButton").pack(side="right")
        ttk.Button(actions, text="取消", command=self.destroy).pack(side="right", padx=(0, 8))
        self.select_member()
        self.bind("<Escape>", lambda _event: self.destroy())
        self.grab_set()
        name_entry.focus_set()

    def select_member(self, _event=None):
        index = self.member_picker.current() - 1
        member = self.members[index] if index >= 0 else None
        self.name.set(member.name if member else self.default_name)
        self.remove_button.configure(state="normal" if member else "disabled")
        self.error.set("")
        for (coordinates, mount), end in zip(self.ends, (member.a, member.b) if member else (None, None)):
            mount.set(end.mount if end else CHASSIS)
            for axis, entry in enumerate(coordinates):
                entry.set(str(end.xyz[axis] * 1000.) if end else "")

    def apply(self):
        index = self.member_picker.current() - 1
        members = list(self.members)
        try:
            ends = []
            for label, (coordinates, mount) in zip("AB", self.ends):
                xyz = tuple(GeometrySettings.value(entry.get(), f"端點 {label} {axis}") / 1000.
                            for axis, entry in zip("XYZ", coordinates))
                ends.append(DeformableEnd(xyz, mount.get()))
            member = DeformableMember(self.name.get().strip(), *ends)
            if index < 0:
                members.append(member)
            else:
                members[index] = member
            self.on_apply(tuple(members))
        except (OSError, ValueError) as error:
            self.error.set(str(error))
            return
        self.destroy()

    def remove(self):
        index = self.member_picker.current() - 1
        if index >= 0:
            try:
                self.on_apply((*self.members[:index], *self.members[index + 1:]))
            except (OSError, ValueError) as error:
                self.error.set(str(error))
                return
            self.destroy()
