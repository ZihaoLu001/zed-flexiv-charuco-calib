"""Intrinsics coverage assessment (X/Y/Size/Skew). Core is pure numpy; heatmap needs cv2."""
import numpy as np
import pytest
from conftest import R_axis_angle

from zfcc import se3
from zfcc.config import CoverageGates
from zfcc.coverage import coverage_report
from zfcc.detect import Detection

W, H = 1280, 720


def _grid(cx, cy, half_w, half_h, nx=8, ny=6):
    xs = np.linspace(cx - half_w, cx + half_w, nx)
    ys = np.linspace(cy - half_h, cy + half_h, ny)
    pts = np.array([[x, y] for x in xs for y in ys], dtype=np.float32)
    return Detection(charuco_corners=pts.reshape(-1, 1, 2),
                     charuco_ids=np.arange(len(pts)).reshape(-1, 1),
                     n_corners=len(pts), laplacian_var=120.0)


def _good_views():
    dets, poses = [], []
    rng = np.random.default_rng(0)
    # board roams the whole frame, sizes vary (near/far), tilts vary
    for _ in range(16):
        cx = rng.uniform(0.2 * W, 0.8 * W)
        cy = rng.uniform(0.2 * H, 0.8 * H)
        scale = rng.uniform(0.12, 0.34)
        dets.append(_grid(cx, cy, scale * W, scale * H))
        tilt = rng.uniform(0, 35)
        poses.append(se3.Rt_to_T(R_axis_angle([1, 0.3, 0], tilt), np.array([0, 0, 0.6])))
    return dets, poses


def test_good_coverage_passes():
    dets, poses = _good_views()
    rep = coverage_report(dets, (W, H), board_poses_T=poses, gates=CoverageGates())
    assert rep.verdict == "PASS", rep.reasons
    assert rep.x_center_fill > 0.5 and rep.y_center_fill > 0.5
    assert rep.corner_area_fill >= 0.6
    assert rep.size_ratio >= 1.5
    assert rep.max_skew_deg >= 20


def test_clustered_views_warn():
    # every view a small board in the center, same size, fronto-parallel -> poor coverage
    dets = [_grid(W / 2, H / 2, 0.08 * W, 0.08 * H) for _ in range(12)]
    poses = [se3.Rt_to_T(np.eye(3), np.array([0, 0, 0.6])) for _ in range(12)]
    rep = coverage_report(dets, (W, H), board_poses_T=poses, gates=CoverageGates())
    assert rep.verdict == "WARN"
    text = " ".join(rep.reasons).lower()
    assert "width" in text or "height" in text or "cells" in text or "distance" in text or "tilt" in text


def _spread_views(scale=0.20):
    """Views with GOOD x/y/area coverage at a FIXED apparent size (isolates the size-ratio gate)."""
    dets = []
    for cx in (0.15 * W, 0.5 * W, 0.85 * W):
        for cy in (0.18 * H, 0.5 * H, 0.82 * H):
            dets.append(_grid(cx, cy, scale * W, scale * H))
    return dets


def test_size_ratio_gate_isolated():
    # good X/Y/area coverage but every board the SAME apparent size -> only the size-ratio gate fires
    dets = _spread_views()
    rep = coverage_report(dets, (W, H), board_poses_T=None, gates=CoverageGates())
    assert rep.size_ratio < 1.5
    assert rep.x_center_fill > 0.5 and rep.y_center_fill > 0.5 and rep.corner_area_fill >= 0.6
    assert rep.verdict == "WARN"
    assert any("size ratio" in r.lower() for r in rep.reasons)


def test_skew_gate_isolated_and_angle_correct():
    # good coverage but all views nearly fronto-parallel (small tilt) -> only the skew gate fires,
    # and the reported max_skew must match the injected tilt
    dets = _spread_views(scale=0.18)
    sizes = [_grid(0.5 * W, 0.5 * H, s * W, s * H) for s in (0.12, 0.34)]  # add size variety
    dets += sizes
    tilt = 8.0
    poses = [se3.Rt_to_T(R_axis_angle([1, 0, 0], tilt), np.array([0, 0, 0.6])) for _ in dets]
    rep = coverage_report(dets, (W, H), board_poses_T=poses, gates=CoverageGates())
    assert abs(rep.max_skew_deg - tilt) < 0.5          # skew angle computed correctly
    assert rep.max_skew_deg < 20
    assert rep.verdict == "WARN"
    assert any("tilt" in r.lower() for r in rep.reasons)


def test_skew_skipped_without_poses():
    dets, _ = _good_views()
    rep = coverage_report(dets, (W, H), board_poses_T=None, gates=CoverageGates())
    assert rep.max_skew_deg == -1.0   # not assessed


def test_no_detections_fails():
    rep = coverage_report([], (W, H))
    assert rep.verdict == "FAIL"


def test_heatmap_render(tmp_path):
    pytest.importorskip("cv2")
    import cv2

    from zfcc.coverage import render_coverage_heatmap
    dets, poses = _good_views()
    rep = coverage_report(dets, (W, H), board_poses_T=poses)
    p = render_coverage_heatmap(rep, str(tmp_path / "cov.png"))
    img = cv2.imread(p)
    assert img is not None and img.shape[2] == 3
