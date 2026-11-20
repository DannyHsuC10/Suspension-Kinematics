from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from JacobianKinematicsLab.model import DeformableEnd, DeformableMember, Drive, EXAMPLES, Link, Model
from JacobianKinematicsLab.model import ROCKER_ANCHORS, load_model, make_motion
from JacobianKinematicsLab.project import load_project, save_project
from JacobianKinematicsLab.results import damper_histories
from JacobianKinematicsLab.solver import solve
from JacobianKinematicsLab.suspension import _roll_difference_percent, sweep_dampers


DAMPER = DeformableMember("DAMPER_FL", DeformableEnd((.02, .31, .37), "FL"),
                          DeformableEnd((.02, .15, .45)))


class DeformableMemberTests(unittest.TestCase):
    def test_endpoints_follow_their_own_mounts_and_allow_length_change(self):
        path = EXAMPLES / "hardpoints.csv"
        baseline = load_model(path)
        cross = DeformableMember("CROSS_DAMPER", DeformableEnd((.02, .31, .37), "RL"),
                                 DeformableEnd((.02, -.31, .37), "RR"))
        model = load_model(path, deformable_members=(DAMPER, cross))
        motion = make_motion(model, samples=5, amplitudes_mm={"RR_WC_Z": -10.})
        result = solve(model, motion)
        np.testing.assert_allclose(result.position[:, :len(baseline.point_names)],
                                   solve(baseline, motion).position, atol=1e-10)
        self.assertFalse(model.links[-2].constrained)
        ids = {name: i for i, name in enumerate(result.point_names)}
        for point, corner in (("DAMPER_FL_A", "FL"), ("CROSS_DAMPER_A", "RL"), ("CROSS_DAMPER_B", "RR")):
            i = ids[point]
            self.assertGreater(np.linalg.norm(result.displacement[2, i]), 1e-4)
            for suffix in ROCKER_ANCHORS:
                anchor = ids[f"{corner}_{suffix}"]
                distance = np.linalg.norm(result.position[:, i] - result.position[:, anchor], axis=1)
                np.testing.assert_allclose(distance, distance[0], atol=1e-10)
        a, b = ids["DAMPER_FL_A"], ids["DAMPER_FL_B"]
        np.testing.assert_array_equal(result.displacement[:, b], 0.)
        length = np.linalg.norm(result.position[:, b] - result.position[:, a], axis=1)
        self.assertGreater(length[0] - length[2], .001)
        self.assertAlmostEqual(length[0], length[-1])

    def test_project_round_trip_preserves_mounts_and_reference_points(self):
        table, model, tires, motion = load_project(EXAMPLES / "hardpoints.csv")
        self.assertEqual(model.deformable_members, ())
        with TemporaryDirectory() as directory:
            path = Path(directory) / "damper.csv"
            save_project(path, table, table.points, model.pushrod_mounts, tires, motion,
                         (DAMPER,), update_source=False)
            saved_table, saved_model, _, _ = load_project(path)
        self.assertEqual(saved_table.points, table.points)
        self.assertEqual(saved_model.deformable_members, (DAMPER,))
        np.testing.assert_array_equal(saved_model.reference[-2:], (DAMPER.a.xyz, DAMPER.b.xyz))

    def test_roll_sweep_overlays_opposite_wheel_differences_at_equal_heave(self):
        right = DeformableMember("DAMPER_FR", DeformableEnd((.02, -.31, .37), "FR"),
                                 DeformableEnd((.02, -.15, .45)))
        model = load_model(EXAMPLES / "hardpoints.csv", deformable_members=(DAMPER, right))
        roll = sweep_dampers(model, "F", "ROLL", (0., 30.), 8.)
        left, right = roll.wheel_travel_mm.transpose(1, 0, 2)
        np.testing.assert_allclose(left + right, 8.)
        np.testing.assert_allclose(left - right, (roll.input_mm, -roll.input_mm))
        self.assertEqual(roll.member_names, ("DAMPER_FL", "DAMPER_FR"))
        self.assertGreater(roll.compression_mm[0, 0, -1], 0.)
        self.assertLess(roll.compression_mm[0, 1, -1], 0.)
        np.testing.assert_allclose(roll.compression_mm[0], roll.compression_mm[1, ::-1], atol=1e-7)
        stroke = np.abs(roll.compression_mm)
        expected = np.abs(stroke[:, 0] - stroke[:, 1]) / stroke.mean(axis=1) * 100.
        np.testing.assert_allclose(roll.roll_difference_pct, expected)

    def test_roll_difference_uses_magnitude_and_handles_zero_travel(self):
        left = np.array((0., 1e-9, 10., 10., 0.))
        right = np.array((0., 0., -10., -8., -10.))
        expected = (0., 0., 0., 200. / 9., 200.)
        np.testing.assert_allclose(_roll_difference_percent(left, right), expected)
        np.testing.assert_allclose(_roll_difference_percent(right, left), expected)

    def test_heave_sweep_uses_wheel_displacement_sum_and_keeps_other_axle_at_zero(self):
        rear = DeformableMember("DAMPER_RL", DeformableEnd((.02, .31, .37), "RL"), DAMPER.b)
        model = load_model(EXAMPLES / "hardpoints.csv", deformable_members=(DAMPER, rear))
        heave = sweep_dampers(model, "R", "HEAVE", (-20., 30.), -6.)
        self.assertIsNone(heave.roll_difference_pct)
        left, right = heave.wheel_travel_mm[0]
        np.testing.assert_allclose(left + right, heave.input_mm, atol=1e-12)
        np.testing.assert_allclose(left - right, -6.)
        np.testing.assert_allclose(heave.compression_mm[0], 0., atol=1e-7)
        self.assertGreater(np.ptp(heave.compression_mm[1]), 1.)

    def test_axial_acceleration_accounts_for_axis_rotation(self):
        model = Model("slider", ("A", "B"), np.array(((0., 0., 0.), (.1, 0., 0.))), ("A",),
                      (Link(DAMPER.name, "A", "B", constrained=False),),
                      (Drive("x", "B", (1., 0., 0.), locked=True),
                       Drive("y", "B", (0., 1., 0.), amplitude_mm=100., profile="ramp"),
                       Drive("z", "B", (0., 0., 1.), locked=True)),
                      deformable_members=(DAMPER,))
        result = solve(model, make_motion(model, samples=5, duration=1.))
        history = damper_histories(model, result)[DAMPER.name]
        length = .1 * np.sqrt(1. + result.time**2)
        np.testing.assert_allclose(history.compression, .1 - length, atol=1e-14)
        np.testing.assert_allclose(history.velocity, -.01 * result.time / length, atol=1e-14)
        np.testing.assert_allclose(history.acceleration, -.0001 / length**3, atol=1e-14)

    def test_zero_length_sample_does_not_invalidate_node_motion(self):
        model = Model("slider", ("A", "B"), np.array(((0., 0., 0.), (.1, 0., 0.))), ("A",),
                      (Link(DAMPER.name, "A", "B", constrained=False),),
                      (Drive("x", "B", (1., 0., 0.), amplitude_mm=-100., profile="ramp"),
                       Drive("y", "B", (0., 1., 0.), locked=True),
                       Drive("z", "B", (0., 0., 1.), locked=True)),
                      deformable_members=(DAMPER,))
        result = solve(model, make_motion(model, samples=3))
        np.testing.assert_array_equal(result.position[-1, 0], result.position[-1, 1])
        self.assertTrue(np.isnan(result.elevation_deg[-1, 0]))
        self.assertTrue(np.isnan(result.angular_velocity[-1, 0]).all())
        history = damper_histories(model, result)[DAMPER.name]
        np.testing.assert_allclose(history.compression[-1], .1)
        np.testing.assert_allclose(history.velocity[:-1], .05)
        self.assertTrue(np.isnan(history.velocity[-1]))
        self.assertTrue(np.isnan(history.acceleration[-1]))


if __name__ == "__main__":
    unittest.main()
