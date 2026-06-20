"""Pose-diversity / degeneracy gates.

This is the explicit anti-"old-calibration-bug" core: the previous extrinsic was solved from 6
COPLANAR markers (a degenerate set, weak in Z/tilt). These gates quantify rotation-axis spread,
coplanarity and translation span, and REFUSE a degenerate pose set before any solve is trusted.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import se3
from .config import DiversityGates

__all__ = ["DiversityReport", "rotation_axis_spread_deg", "n_distinct_rotation_axes",
           "coplanarity_index", "min_interpose_rotation_deg", "translation_span_m",
           "distinct_depths", "assess_diversity"]


def _axes(poses_T):
    out = []
    for T in poses_T:
        ax = se3.rotation_axis(T[:3, :3])
        if np.linalg.norm(ax) > 1e-6:
            out.append(ax * np.sign(ax[np.argmax(np.abs(ax))]))  # canonical hemisphere
    return np.asarray(out) if out else np.zeros((0, 3))


def rotation_axis_spread_deg(poses_T) -> float:
    """Max angle (deg) between any two per-pose rotation axes -- a single-cone set scores low."""
    A = _axes(poses_T)
    if len(A) < 2:
        return 0.0
    m = 0.0
    for i in range(len(A)):
        for j in range(i + 1, len(A)):
            c = np.clip(np.dot(A[i], A[j]), -1.0, 1.0)
            m = max(m, float(np.degrees(np.arccos(abs(c)))))  # abs: axis is sign-ambiguous
    return m


def n_distinct_rotation_axes(poses_T, sep_deg: float = 20.0) -> int:
    """Greedy count of mutually >sep_deg-separated rotation axes."""
    A = _axes(poses_T)
    reps = []
    for a in A:
        if all(np.degrees(np.arccos(np.clip(abs(np.dot(a, r)), -1, 1))) > sep_deg for r in reps):
            reps.append(a)
    return len(reps)


def coplanarity_index(positions) -> float:
    """smallest/largest singular value of the centred Nx3 position matrix.

    ~0 => points lie on a plane (degenerate, like the old set); ~1 => well-spread in 3D."""
    P = np.asarray(positions, dtype=float)
    if len(P) < 3:
        return 0.0
    P = P - P.mean(axis=0, keepdims=True)
    sv = np.linalg.svd(P, compute_uv=False)
    return float(sv[-1] / sv[0]) if sv[0] > 1e-12 else 0.0


def min_interpose_rotation_deg(poses_T) -> float:
    """Smallest relative rotation between any two poses (deg) -- near-0 => redundant poses."""
    if len(poses_T) < 2:
        return 0.0
    m = 1e9
    for i in range(len(poses_T)):
        for j in range(i + 1, len(poses_T)):
            dR = poses_T[i][:3, :3] @ poses_T[j][:3, :3].T
            m = min(m, se3.rotation_angle_deg(dR))
    return float(m)


def translation_span_m(positions) -> float:
    """Range of the position depths/extent -- max pairwise distance (m)."""
    P = np.asarray(positions, dtype=float)
    if len(P) < 2:
        return 0.0
    return float(np.max(np.linalg.norm(P[:, None, :] - P[None, :, :], axis=-1)))


def distinct_depths(board_poses_T, bin_m: float = 0.05) -> int:
    """Count board-to-camera working distances that are mutually separated by >= ``bin_m``.

    The board's depth in the CAMERA frame (``T_cam_board`` z) is what constrains focal length and the
    camera Z of the extrinsic; seeing the board at only one distance is a classic weak-Z set. Uses a
    greedy sorted merge (like ``n_distinct_rotation_axes``) so the count reflects genuine separation
    and is independent of where depths fall relative to fixed bin edges. Returns 0 if no board poses.
    """
    if board_poses_T is None or len(board_poses_T) == 0:
        return 0
    depths = sorted(float(T[2, 3]) for T in board_poses_T)
    reps = []
    for d in depths:
        if all(abs(d - r) >= bin_m for r in reps):
            reps.append(d)
    return len(reps)


@dataclass
class DiversityReport:
    n_poses: int
    rotation_axes: int
    rotation_axis_spread_deg: float
    max_interpose_rotation_deg: float
    coplanarity_index: float
    translation_span_m: float
    verdict: str                      # PASS / WARN / FAIL
    distinct_depths: int = -1         # -1 = not assessed (board poses not supplied)
    reasons: list = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "n_poses": self.n_poses,
            "rotation_axes": self.rotation_axes,
            "rotation_axis_spread_deg": round(self.rotation_axis_spread_deg, 2),
            "coplanarity_index": round(self.coplanarity_index, 4),
            "translation_span_m": round(self.translation_span_m, 3),
            "distinct_depths": self.distinct_depths,
            "verdict": self.verdict,
            "reasons": self.reasons,
        }


def assess_diversity(flange_poses_T, gates: DiversityGates | None = None,
                     board_poses_T=None) -> DiversityReport:
    """Grade a pose set; the solve refuses to write a calibration if the verdict is FAIL.

    ``board_poses_T`` (list of T_cam_board) is optional but recommended: it enables the
    distinct-working-distance gate (focal length / camera-Z are weak if every view is at one depth).
    """
    g = gates or DiversityGates()
    positions = np.asarray([T[:3, 3] for T in flange_poses_T], dtype=float)
    n = len(flange_poses_T)
    naxes = n_distinct_rotation_axes(flange_poses_T)
    spread = rotation_axis_spread_deg(flange_poses_T)
    # "max interpose rotation" is the useful spread; "min" guards redundancy
    maxrot = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            maxrot = max(maxrot, se3.rotation_angle_deg(
                flange_poses_T[i][:3, :3] @ flange_poses_T[j][:3, :3].T))
    cop = coplanarity_index(positions)
    span = translation_span_m(positions)
    ndepths = distinct_depths(board_poses_T, g.depth_bin_m) if board_poses_T is not None else -1

    reasons, verdict = [], "PASS"

    def fail(msg):
        nonlocal verdict
        verdict = "FAIL"
        reasons.append("FAIL: " + msg)

    def warn(msg):
        nonlocal verdict
        if verdict != "FAIL":
            verdict = "WARN"
        reasons.append("WARN: " + msg)

    if n < g.min_poses:
        fail(f"only {n} poses (< {g.min_poses})")
    elif n < g.min_poses_pass:
        warn(f"{n} poses (< {g.min_poses_pass} recommended)")
    if naxes < g.min_rotation_axes:
        fail(f"only {naxes} distinct rotation axes (< {g.min_rotation_axes}); near-parallel set")
    if cop < g.coplanarity_index_fail:
        fail(f"coplanarity index {cop:.4f} < {g.coplanarity_index_fail} -- near-planar (the old-bug mode)")
    if maxrot < g.min_interpose_rotation_deg_pass:
        if maxrot < g.min_interpose_rotation_deg_warn:
            fail(f"max inter-pose rotation {maxrot:.1f} deg too small")
        else:
            warn(f"max inter-pose rotation {maxrot:.1f} deg (< {g.min_interpose_rotation_deg_pass} ideal)")
    if ndepths >= 0 and ndepths < g.min_distinct_depths:
        fail(f"only {ndepths} distinct board-to-camera depths (< {g.min_distinct_depths}); "
             f"vary the working distance (weak-Z set)")

    return DiversityReport(n_poses=n, rotation_axes=naxes, rotation_axis_spread_deg=spread,
                           max_interpose_rotation_deg=maxrot, coplanarity_index=cop,
                           translation_span_m=span, distinct_depths=ndepths,
                           verdict=verdict, reasons=reasons)
