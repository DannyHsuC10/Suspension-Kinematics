from types import SimpleNamespace
import unittest

import numpy as np

from JacobianKinematicsLab.interference import INTERFERENCE_TOLERANCE_M, find_interferences
from JacobianKinematicsLab.model import EXAMPLES, Drive, Link, Model, make_motion
from JacobianKinematicsLab.project import load_project
from JacobianKinematicsLab.solver import solve


def rod_history(reference, *samples, links=None, standard=False, deformable=()):
    names = ("A0", "A1", "B0", "B1")
    if links is None:
        links = (Link("A", *names[:2]), Link("B", *names[2:]))
    model = SimpleNamespace(point_names=names, reference=np.asarray(reference, dtype=float), links=links,
                            standard_suspension=standard, deformable_members=deformable)
    result = SimpleNamespace(point_names=names, position=np.asarray(samples or (reference,), dtype=float))
    return model, result


class RodInterferenceTests(unittest.TestCase):
    def test_detects_a_moving_rod_crossing_at_the_solved_midpoint(self):
        reference = np.array(((-.1, 0, 0), (.1, 0, 0), (0, -.1, -.01), (0, .1, -.01)))
        drives = tuple(
            Drive(f"{point}_{axis}", point, direction, amplitude_mm=10. if axis == "Z" else 0., locked=axis != "Z")
            for point in ("B0", "B1")
            for axis, direction in zip("XYZ", ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
        )
        model = Model("Crossing", ("A0", "A1", "B0", "B1"), reference, ("A0", "A1"),
                      (Link("A", "A0", "A1"), Link("B", "B0", "B1")), drives, samples=3)
        result = solve(model, make_motion(model))
        hit, = find_interferences(model, result)
        self.assertEqual((hit.first_sample, hit.last_sample, hit.sample_count), (1, 1, 1))
        self.assertEqual(result.time[hit.first_sample], 1.)

    def test_reports_crossing_samples_without_counting_skew_or_extended_lines(self):
        crossing = ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
        skew = (*crossing[:2], (0, -1, .1), (0, 1, .1))
        extended = (*crossing[:2], (0, 1, 0), (0, 2, 0))
        near = (*crossing[:2], (0, -1, INTERFERENCE_TOLERANCE_M / 2), (0, 1, INTERFERENCE_TOLERANCE_M / 2))
        hits = find_interferences(*rod_history(skew, skew, crossing, extended, near))
        self.assertEqual(len(hits), 1)
        hit = hits[0]
        self.assertEqual((hit.rod_a, hit.rod_b), ("A", "B"))
        self.assertEqual((hit.first_sample, hit.last_sample, hit.sample_count), (1, 3, 2))
        self.assertAlmostEqual(hit.minimum_distance_m, 0.)
        self.assertEqual(find_interferences(*rod_history(skew)), ())
        self.assertEqual(find_interferences(*rod_history(extended)), ())

    def test_parallel_and_collapsed_segments(self):
        offset = INTERFERENCE_TOLERANCE_M / 2
        parallel = ((-1, 0, 0), (1, 0, 0), (-2, offset, 0), (0, offset, 0))
        hit, = find_interferences(*rod_history(parallel))
        self.assertAlmostEqual(hit.minimum_distance_m, offset)
        collapsed = (*parallel[:2], (0, 0, 0), (0, 0, 0))
        hit, = find_interferences(*rod_history(parallel, collapsed))
        self.assertEqual(hit.minimum_distance_m, 0.)

    def test_ignores_intentional_endpoint_contacts_but_reports_interior_overlap(self):
        joint = ((0, 0, 0), (1, 0, 0), (0, 0, 0), (0, 1, 0))
        crossing = (*joint[:2], (.5, -1, 0), (.5, 1, 0))
        self.assertEqual(find_interferences(*rod_history(joint)), ())
        hit, = find_interferences(*rod_history(joint, joint, crossing))
        self.assertEqual((hit.first_sample, hit.sample_count), (1, 1))

        fully_overlapping = (*joint[:2], *joint[:2])
        hit, = find_interferences(*rod_history(fully_overlapping))
        self.assertEqual(hit.minimum_distance_m, 0.)

        shared = (Link("A", "A0", "A1"), Link("B", "A0", "B1"))
        self.assertEqual(find_interferences(*rod_history(joint, links=shared)), ())

    def test_only_checks_physical_members_of_standard_suspension(self):
        crossing = ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
        links = (Link("FL_UCA_F-UBJ", "A0", "A1"), Link("FL_UBJ-TOE", "B0", "B1"))
        self.assertEqual(find_interferences(*rod_history(crossing, links=links, standard=True)), ())

        damper = SimpleNamespace(name="Damper")
        links = (links[0], Link(damper.name, "B0", "B1", constrained=False))
        self.assertEqual(find_interferences(*rod_history(crossing, links=links)), ())
        hit, = find_interferences(*rod_history(crossing, links=links, standard=True, deformable=(damper,)))
        self.assertEqual(hit.rod_b, damper.name)

    def test_bundled_reference_suspension_has_no_centerline_crossing(self):
        _, model, _, settings = load_project(EXAMPLES / "hardpoints.csv")
        result = solve(model, make_motion(model, **settings))
        self.assertEqual(find_interferences(model, result), ())


if __name__ == "__main__":
    unittest.main()
