"""Check the physical rod centerlines at every solved pose."""

from dataclasses import dataclass
from itertools import combinations, product
from math import isfinite

import numpy as np

from .model import CORNERS


INTERFERENCE_TOLERANCE_M = 1e-5
_COINCIDENT_JOINT_TOLERANCE_M = 1e-8
_STANDARD_RODS = {
    f"{corner}_{suffix}"
    for corner in CORNERS
    for suffix in ("UCA_F-UBJ", "UCA_R-UBJ", "LCA_F-LBJ", "LCA_R-LBJ", "RACK-TOE", "PUSH_ROD")
} | {"FRONT_RACK_BAR"}


@dataclass(frozen=True)
class RodInterference:
    rod_a: str
    rod_b: str
    first_sample: int
    last_sample: int
    sample_count: int
    minimum_distance_m: float


def _dot(a, b):
    return np.einsum("ij,ij->i", a, b)


def _project(offset, direction, length_squared):
    return np.clip(np.divide(
        _dot(offset, direction), length_squared, out=np.zeros_like(length_squared),
        where=length_squared > 0,
    ), 0, 1)


def _segment_distances(a0, a1, b0, b1, joint_pairs, joint_tolerance):
    """Minimize the centerline distance over two closed segment parameters."""
    u, v, offset = a1 - a0, b1 - b0, a0 - b0
    u_squared, v_squared = _dot(u, u), _dot(v, v)
    distances_squared = np.full(len(a0), np.inf)
    current_joints = ()
    if joint_pairs:
        u_length, v_length = np.sqrt(u_squared), np.sqrt(v_squared)
        current_joints = [
            (end_a, end_b, np.linalg.norm((a0, a1)[end_a] - (b0, b1)[end_b], axis=1) <= joint_tolerance)
            for end_a, end_b in joint_pairs
        ]

    def consider(s, t):
        valid = (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
        for end_a, end_b, coincident in current_joints:
            at_joint = ((np.abs(s - end_a) * u_length <= joint_tolerance)
                        & (np.abs(t - end_b) * v_length <= joint_tolerance))
            valid = valid & ~(coincident & at_joint)
        separation = offset + np.asarray(s)[..., None] * u - np.asarray(t)[..., None] * v
        np.minimum(distances_squared, _dot(separation, separation), out=distances_squared, where=valid)

    # The closest points lie on a parameter-square edge or at its stationary point.
    consider(0., _project(offset, v, v_squared))
    consider(1., _project(offset + u, v, v_squared))
    consider(_project(-offset, u, u_squared), 0.)
    consider(_project(v - offset, u, u_squared), 1.)
    normal = np.cross(u, v)
    normal_squared = _dot(normal, normal)
    nonparallel = normal_squared > 0
    s = np.divide(_dot(np.cross(v, offset), normal), normal_squared,
                  out=np.full_like(normal_squared, np.nan), where=nonparallel)
    t = np.divide(_dot(np.cross(u, offset), normal), normal_squared,
                  out=np.full_like(normal_squared, np.nan), where=nonparallel)
    consider(s, t)
    if current_joints:
        # Fully overlapping segments may have intentional joints at both ends.
        consider(.5, _project(offset + .5 * u, v, v_squared))
    return np.sqrt(distances_squared)


def find_interferences(model, result, *, tolerance_m=INTERFERENCE_TOLERANCE_M):
    """Report each pair whose two finite centerlines meet within tolerance at a solved sample.

    The model supplies reference geometry and physical rod names. Result positions are in metres.
    Report sample indices are zero-based. Shared named joints and endpoint-only contacts at
    coincident reference joints are ignored.
    """
    if not isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError("Interference tolerance must be finite and nonnegative.")
    point_index = {name: i for i, name in enumerate(result.point_names)}
    reference_index = {name: i for i, name in enumerate(model.point_names)}
    deformable = {member.name for member in model.deformable_members}
    rods = [link for link in model.links if link.name in deformable or
            (link.name in _STANDARD_RODS if model.standard_suspension else link.constrained)]
    positions = result.position
    if len(rods) < 2 or not len(positions):
        return ()

    point_min, point_max = positions.min(axis=0), positions.max(axis=0)
    starts = [point_index[link.a] for link in rods]
    ends = [point_index[link.b] for link in rods]
    rod_min = np.minimum(point_min[starts], point_min[ends])
    rod_max = np.maximum(point_max[starts], point_max[ends])
    joint_tolerance = min(tolerance_m, _COINCIDENT_JOINT_TOLERANCE_M)
    hits = []
    for i, j in combinations(range(len(rods)), 2):
        rod_a, rod_b = rods[i], rods[j]
        if {rod_a.a, rod_a.b} & {rod_b.a, rod_b.b}:
            continue
        if np.any(np.maximum(rod_min[i], rod_min[j]) - np.minimum(rod_max[i], rod_max[j]) > tolerance_m):
            continue

        a0, a1 = positions[:, starts[i]], positions[:, ends[i]]
        b0, b1 = positions[:, starts[j]], positions[:, ends[j]]
        box_min = np.maximum(np.minimum(a0, a1), np.minimum(b0, b1))
        box_max = np.minimum(np.maximum(a0, a1), np.maximum(b0, b1))
        candidates = np.flatnonzero(np.all(box_min - box_max <= tolerance_m, axis=1))
        if not len(candidates):
            continue
        reference_a = model.reference[[reference_index[rod_a.a], reference_index[rod_a.b]]]
        reference_b = model.reference[[reference_index[rod_b.a], reference_index[rod_b.b]]]
        joint_pairs = [
            (a, b) for a, b in product(range(2), repeat=2)
            if np.linalg.norm(reference_a[a] - reference_b[b]) <= joint_tolerance
        ]
        distances = _segment_distances(a0[candidates], a1[candidates], b0[candidates], b1[candidates],
                                       joint_pairs, joint_tolerance)
        interfering = distances <= tolerance_m
        samples = candidates[interfering]
        if len(samples):
            hits.append(RodInterference(rod_a.name, rod_b.name, int(samples[0]), int(samples[-1]),
                                        len(samples), float(distances[interfering].min())))
    return tuple(sorted(hits, key=lambda hit: (hit.first_sample, hit.rod_a, hit.rod_b)))
