"""Nonlinear refinement of the extrinsic (optional scipy dep)."""
import numpy as np
import pytest
from conftest import make_diverse_flange_poses, synth_board_poses

from zfcc import se3
from zfcc.handeye import axxb_residuals, solve_eye_to_hand

pytest.importorskip("scipy")
pytest.importorskip("cv2")
from zfcc.refine import refine_extrinsic


def _seed(flange, boards):
    T0 = solve_eye_to_hand(flange, boards).T_primary
    Tfb0 = axxb_residuals(T0, flange, boards)["T_flange_board"]
    return T0, Tfb0


def test_refine_recovers_ground_truth_noisefree(gt_extrinsics):
    T_bc, T_fb = gt_extrinsics
    flange = make_diverse_flange_poses(20)
    boards = synth_board_poses(flange, T_bc, T_fb)
    T0, Tfb0 = _seed(flange, boards)
    res = refine_extrinsic(T0, Tfb0, flange, boards)
    assert res.converged
    assert np.linalg.norm(res.T_base_camera[:3, 3] - T_bc[:3, 3]) * 1000 < 0.5
    assert res.rms_translation_mm < 0.05


def _perturb(T, dt_m, dr_deg):
    """Nudge a transform by a fixed translation + rotation to make a deliberately degraded seed."""
    import cv2
    out = T.copy()
    out[:3, 3] = out[:3, 3] + np.array([dt_m, -dt_m, dt_m])
    dR, _ = cv2.Rodrigues(np.radians(dr_deg) * np.array([1.0, 1.0, 0.0]) / np.sqrt(2))
    out[:3, :3] = dR @ out[:3, :3]
    return out


def test_refine_improves_a_degraded_seed(gt_extrinsics):
    """The point of refinement: from a perturbed seed it must REDUCE the pose-consistency residual
    (a no-op return, or making it worse, would fail this)."""
    T_bc, T_fb = gt_extrinsics
    flange = make_diverse_flange_poses(24)
    boards = synth_board_poses(flange, T_bc, T_fb, noise_m=0.0008, noise_deg=0.06, seed=9)
    T0, Tfb0 = _seed(flange, boards)
    T0_bad = _perturb(T0, 0.004, 0.4)              # 4 mm + 0.4 deg off the closed-form seed
    seed_resid_max = axxb_residuals(T0_bad, flange, boards)["translation_mm_max"]
    res = refine_extrinsic(T0_bad, Tfb0, flange, boards)
    assert res.converged
    assert res.delta_from_init_mm > 0.0                       # it actually moved (not a no-op)
    assert res.rms_translation_mm < seed_resid_max            # and reduced the residual
    assert np.linalg.norm(res.T_base_camera[:3, 3] - T_bc[:3, 3]) * 1000 < 5.0


def test_refine_reports_finite_covariance_under_noise(gt_extrinsics):
    T_bc, T_fb = gt_extrinsics
    flange = make_diverse_flange_poses(20)
    boards = synth_board_poses(flange, T_bc, T_fb, noise_m=0.0005, noise_deg=0.04, seed=3)
    T0, Tfb0 = _seed(flange, boards)
    res = refine_extrinsic(T0, Tfb0, flange, boards)
    d = res.as_dict()
    assert d["covariance_available"] is True
    assert set(d["param_std"]) == {"rx", "ry", "rz", "tx", "ty", "tz"}
    assert all(np.isfinite(v) and v >= 0 for v in d["param_std"].values())


def test_refine_flags_unavailable_covariance_noisefree(gt_extrinsics):
    """On a (near-)zero-residual fit the Jacobian is singular -> covariance is genuinely unavailable,
    and that must be reported explicitly, never a silently-empty std dict passed off as certainty."""
    T_bc, T_fb = gt_extrinsics
    flange = make_diverse_flange_poses(20)
    boards = synth_board_poses(flange, T_bc, T_fb)   # noise-free
    T0, Tfb0 = _seed(flange, boards)
    res = refine_extrinsic(T0, Tfb0, flange, boards)
    # either a finite covariance, or explicitly flagged unavailable with an empty dict -- never NaN/inf
    assert res.covariance_available == bool(res.param_std)
    assert all(np.isfinite(v) for v in res.param_std.values())


def test_refine_needs_min_poses(gt_extrinsics):
    T_bc, T_fb = gt_extrinsics
    flange = make_diverse_flange_poses(2)
    boards = synth_board_poses(flange, T_bc, T_fb)
    with pytest.raises(ValueError):
        refine_extrinsic(T_bc, T_fb, flange, boards)
