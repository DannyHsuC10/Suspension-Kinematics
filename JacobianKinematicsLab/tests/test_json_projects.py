import json
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from JacobianKinematicsLab.model import EXAMPLES, DeformableEnd, DeformableMember, load_model, make_motion, read_hardpoints
from JacobianKinematicsLab.project import load_project, save_project
from JacobianKinematicsLab.solver import solve
from JacobianKinematicsLab.wheel_frame import TireSetup


class JsonProjectTests(unittest.TestCase):
    def test_web_suspension_project_restores_settings_and_saves_back_to_json(self):
        points = read_hardpoints(EXAMPLES / "hardpoints.csv")
        mounts = {"FL": "LCA", "FR": "UCA", "RL": "UCA", "RR": "UCA"}
        tire = TireSetup(width_mm=205., radius_mm=215., toe_deg=None, camber_deg=-1.5)
        member = DeformableMember("DAMPER_FL", DeformableEnd((.02, .31, .37), "FL"),
                                  DeformableEnd((.02, .15, .45)))
        motion = {"duration": 1.5, "samples": 5,
                  "amplitudes_mm": {"FL_WC_Z": 12., "FR_WC_Z": 0., "RL_WC_Z": 0., "RR_WC_Z": 0., "FRONT_RACK": 3.},
                  "profiles": {"FL_WC_Z": "ramp", "FR_WC_Z": "pulse", "RL_WC_Z": "pulse",
                               "RR_WC_Z": "pulse", "FRONT_RACK": "sine"}}
        project = {"version": 1, "points": points, "pushrod_mounts": mounts, "tires": {"FL": asdict(tire)},
                   "motion": motion, "deformable_members": [asdict(member)]}

        with TemporaryDirectory() as directory:
            path = Path(directory) / "web-export.JSON"
            path.write_text(json.dumps(project), encoding="utf-8-sig")
            source, model, tires, options = load_project(path)
            self.assertEqual(source.points, points)
            self.assertEqual(source.content, path.read_bytes())
            self.assertEqual(dict(model.pushrod_mounts), mounts)
            self.assertEqual(tires["FL"], tire)
            self.assertEqual(options, motion)
            self.assertEqual(model.deformable_members, (member,))

            result = solve(model, make_motion(model, **options))
            wheel = model.point_names.index("FL_WC")
            np.testing.assert_allclose(result.displacement[:, wheel, 2], np.linspace(0., .012, 5), atol=1e-10)

            x, y, z = source.points["FL_RACK"]
            edited_points = dict(source.points, FL_RACK=(x + .001, y, z))
            options["duration"] = 2.5
            saved = save_project(path, source, edited_points, model.pushrod_mounts, tires, options,
                                 model.deformable_members, update_source=True)
            reopened, reopened_model, reopened_tires, reopened_options = load_project(path)
            self.assertEqual(saved.content, path.read_bytes())
            self.assertEqual(reopened.points, edited_points)
            self.assertEqual(dict(reopened_model.pushrod_mounts), mounts)
            self.assertEqual(reopened_model.deformable_members, (member,))
            self.assertEqual(reopened_tires, tires)
            self.assertEqual(reopened_options, options)

    def test_embedded_web_mechanism_survives_geometry_edits_and_save(self):
        mechanism = json.loads((EXAMPLES / "pushrod_rocker.json").read_text(encoding="utf-8"))
        points = {"CHASSIS_A": (-.05, 0., 0.), "CHASSIS_B": (.05, 0., 0.),
                  "ROCKER_TIP": (0., .1, 0.), "PUSHROD_SLIDER": (0., .1, -.2)}
        motion = {"duration": 1., "samples": 5, "amplitudes_mm": {"SLIDER_Z": 10.},
                  "profiles": {"SLIDER_Z": "ramp"}}
        project = {"version": 1, "points": points, "pushrod_mounts": {}, "tires": {}, "motion": motion,
                   "deformable_members": [], "mechanism": mechanism}

        with TemporaryDirectory() as directory:
            path = Path(directory) / "rocker.json"
            path.write_text(json.dumps(project), encoding="utf-8")
            source, model, tires, options = load_project(path)
            self.assertEqual(source.mechanism, mechanism)
            self.assertEqual(model.name, mechanism["name"])
            self.assertEqual(options, motion)
            self.assertFalse(model.standard_suspension)

            edited_points = dict(source.points, PUSHROD_SLIDER=(0., .1, -.205))
            edited_model = load_model(path, source.mechanism, points=edited_points)
            self.assertEqual(edited_model.links, model.links)
            result = solve(edited_model, make_motion(edited_model, **options))
            slider = edited_model.point_names.index("PUSHROD_SLIDER")
            self.assertAlmostEqual(result.position[-1, slider, 2], -.195)

            save_project(path, source, edited_points, edited_model.pushrod_mounts, tires, options,
                         update_source=True)
            reopened, reopened_model, _, _ = load_project(path)
            self.assertEqual(reopened.mechanism, mechanism)
            self.assertEqual(reopened.points, edited_points)
            self.assertEqual(reopened_model.links, model.links)

    def test_json_save_rejects_external_edits(self):
        points = read_hardpoints(EXAMPLES / "hardpoints.csv")
        with TemporaryDirectory() as directory:
            path = Path(directory) / "web-export.json"
            path.write_text(json.dumps({"version": 1, "points": points}), encoding="utf-8")
            source, model, tires, options = load_project(path)
            external = path.read_bytes() + b"\n"
            path.write_bytes(external)
            with self.assertRaisesRegex(ValueError, "已變更"):
                save_project(path, source, source.points, model.pushrod_mounts, tires, options,
                             update_source=True)
            self.assertEqual(path.read_bytes(), external)
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_json_import_reports_invalid_projects(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            for payload in (b"{", b"\xff", b"[]", b'{"version": true, "points": {"A": [0, 0, 0]}}',
                            b'{"version": 1, "points": {"A": [0, 0, NaN]}}'):
                with self.subTest(payload=payload):
                    path.write_bytes(payload)
                    with self.assertRaises(ValueError):
                        load_project(path)


if __name__ == "__main__":
    unittest.main()
