"""The anti-degeneracy gate must PASS a diverse set and FAIL the old coplanar one -- pure numpy."""
import numpy as np
from conftest import make_coplanar_flange_poses, make_diverse_flange_poses

from zfcc.config import DiversityGates
from zfcc.diversity import (
    assess_diversity,
    coplanarity_index,
    n_distinct_rotation_axes,
    rotation_axis_spread_deg,
)


def test_diverse_set_passes():
    poses = make_diverse_flange_poses(24)
    rep = assess_diversity(poses, DiversityGates())
    assert rep.verdict == "PASS", rep.reasons
    assert rep.rotation_axes >= 3
    assert rep.coplanarity_index > 0.04


def test_coplanar_set_fails_like_old_calibration():
    # 20 poses so the min-poses count gate (< 15) does NOT fire: the FAIL must come from
    # the actual degeneracy gates, not a trivial too-few-views count.
    poses = make_coplanar_flange_poses(20)
    rep = assess_diversity(poses, DiversityGates())
    assert rep.verdict == "FAIL"
    assert not any("poses (<" in r for r in rep.reasons), \
        f"count gate fired -- test no longer exercises the degeneracy gates: {rep.reasons}"
    # the repo's reason-to-exist: the coplanarity gate itself must reject this set on its merits.
    assert any("coplanar" in r.lower() for r in rep.reasons), rep.reasons
    assert rep.coplanarity_index < DiversityGates().coplanarity_index_fail


def test_coplanarity_gate_alone_rejects_old_bug_set():
    """The coplanarity gate must FAIL the near-planar old-bug positions on its OWN merits,
    even if every other gate were satisfied -- this is the anti-degeneracy core."""
    poses = make_coplanar_flange_poses(20)
    rep = assess_diversity(poses, DiversityGates())
    cop_fails = [r for r in rep.reasons if "coplanar" in r.lower()]
    assert cop_fails, rep.reasons
    assert rep.coplanarity_index < DiversityGates().coplanarity_index_fail
    # contrast: a volumetric set clears the same gate
    diverse = assess_diversity(make_diverse_flange_poses(20), DiversityGates())
    assert diverse.coplanarity_index > DiversityGates().coplanarity_index_fail
    assert not any("coplanar" in r.lower() for r in diverse.reasons)


def test_coplanarity_index_zero_for_planar_points():
    pts = np.array([[0, 0, 0.3], [0.1, 0, 0.3], [0, 0.1, 0.3], [0.1, 0.1, 0.3]], dtype=float)
    assert coplanarity_index(pts) < 1e-6


def test_coplanarity_index_positive_for_volume():
    rng = np.random.default_rng(0)
    pts = rng.uniform(0, 0.3, size=(20, 3))
    assert coplanarity_index(pts) > 0.1


def test_n_distinct_axes_counts_separated_axes():
    from conftest import R_axis_angle

    from zfcc import se3
    poses = [se3.Rt_to_T(R_axis_angle(a, 40), np.zeros(3))
             for a in ([1, 0, 0], [0, 1, 0], [0, 0, 1])]
    assert n_distinct_rotation_axes(poses) == 3


def test_rotation_axis_spread_low_for_single_axis():
    from conftest import R_axis_angle

    from zfcc import se3
    poses = [se3.Rt_to_T(R_axis_angle([0, 0, 1], d), np.zeros(3)) for d in (10, 20, 30, 40)]
    assert rotation_axis_spread_deg(poses) < 5.0


def test_distinct_depths_counts_bins():
    from zfcc import se3
    from zfcc.diversity import distinct_depths
    # camera-frame z = 0.40, 0.42, 0.60, 0.90: 0.40~0.42 merge (<5cm apart) -> 3 distinct
    zs = [0.40, 0.42, 0.60, 0.90]
    boards = [se3.Rt_to_T(np.eye(3), np.array([0, 0, z])) for z in zs]
    assert distinct_depths(boards, bin_m=0.05) == 3
    assert distinct_depths(None) == 0


def test_distinct_depths_is_phase_independent():
    """Greedy relative-separation count must depend on SEPARATION, not on where depths fall relative
    to fixed bin edges (the old absolute-grid version was phase-dependent)."""
    from zfcc import se3
    from zfcc.diversity import distinct_depths

    def boards(zs):
        return [se3.Rt_to_T(np.eye(3), np.array([0, 0, z])) for z in zs]
    # two depths 1 cm apart always count as 1, regardless of grid phase
    assert distinct_depths(boards([0.601, 0.611]), bin_m=0.05) == 1
    assert distinct_depths(boards([0.641, 0.651]), bin_m=0.05) == 1   # straddles a 0.05 bin edge
    # two depths 10 cm apart always count as 2, regardless of phase
    assert distinct_depths(boards([0.601, 0.701]), bin_m=0.05) == 2
    assert distinct_depths(boards([0.624, 0.726]), bin_m=0.05) == 2


def test_single_depth_set_fails_even_if_rotations_diverse():
    from zfcc import se3
    poses = make_diverse_flange_poses(20)                       # good rotations
    boards = [se3.Rt_to_T(np.eye(3), np.array([0, 0, 0.6])) for _ in poses]  # all at one depth
    rep = assess_diversity(poses, DiversityGates(), board_poses_T=boards)
    assert rep.verdict == "FAIL"
    assert any("depth" in r.lower() for r in rep.reasons)
    assert rep.distinct_depths == 1


def test_varied_depth_set_passes_depth_gate():
    from zfcc import se3
    poses = make_diverse_flange_poses(20)
    rng = np.random.default_rng(0)
    boards = [se3.Rt_to_T(np.eye(3), np.array([0, 0, 0.4 + 0.5 * rng.random()])) for _ in poses]
    rep = assess_diversity(poses, DiversityGates(), board_poses_T=boards)
    assert rep.distinct_depths >= 3
    assert not any("depth" in r.lower() for r in rep.reasons)
