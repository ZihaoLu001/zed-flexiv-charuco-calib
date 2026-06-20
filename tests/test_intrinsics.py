"""Intrinsics: confirm ChArUco recovers a known K (free solve) and that audit mode reprojects a
correct K with ~0 error and does NOT change it. Uses synthetic projection -- no rendering/hardware.
"""
import numpy as np
import pytest

from zfcc.config import BoardConfig

cv2 = pytest.importorskip("cv2")
from zfcc.board import make_board
from zfcc.detect import Detection
from zfcc.intrinsics import audit_against_factory, calibrate_intrinsics

IMG = (1280, 720)
K_TRUE = np.array([[700.0, 0, 640.0], [0, 700.0, 360.0], [0, 0, 1.0]])


def _rodrigues(ax, deg):
    r = np.asarray(ax, float)
    r = r / np.linalg.norm(r) * np.radians(deg)
    R, _ = cv2.Rodrigues(r)
    return R


def _make_views(board, n=16, seed=0):
    rng = np.random.default_rng(seed)
    objp = np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
    N = objp.shape[0]
    cx, cy = objp[:, 0].mean(), objp[:, 1].mean()
    dets = []
    tries = 0
    while len(dets) < n and tries < 400:
        tries += 1
        R = _rodrigues([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-0.3, 0.3)],
                       rng.uniform(5, 22))
        rvec, _ = cv2.Rodrigues(R)
        Z = rng.uniform(0.7, 1.0)
        tvec = np.array([-cx + rng.uniform(-0.05, 0.05), -cy + rng.uniform(-0.05, 0.05), Z])
        img, _ = cv2.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec.reshape(3, 1), K_TRUE, None)
        pts = img.reshape(-1, 2)
        if pts[:, 0].min() < 5 or pts[:, 0].max() > IMG[0] - 5:
            continue
        if pts[:, 1].min() < 5 or pts[:, 1].max() > IMG[1] - 5:
            continue
        det = Detection(charuco_corners=img.astype(np.float32),
                        charuco_ids=np.arange(N, dtype=np.int32).reshape(-1, 1),
                        n_corners=N, laplacian_var=120.0)
        dets.append(det)
    assert len(dets) >= n, f"only built {len(dets)} in-bounds views"
    return dets


def test_free_solve_recovers_known_K():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    res = calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="free")
    assert res.rms_px < 0.05
    assert abs(res.K[0, 0] - 700) < 1.0 and abs(res.K[1, 1] - 700) < 1.0
    assert abs(res.K[0, 2] - 640) < 1.0 and abs(res.K[1, 2] - 360) < 1.0


def test_audit_mode_reprojects_zero_and_keeps_K_fixed():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    res = calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="audit")
    assert res.mode == "audit"
    assert res.rms_px < 0.05
    assert np.allclose(res.K, K_TRUE, atol=1e-6)   # fixed: not optimized away
    # audit mode's whole premise is the ZED stream is already RECTIFIED (D == 0): the
    # CALIB_FIX_K1|K2|K3 | CALIB_ZERO_TANGENT_DIST flags must hold every coeff at its zero seed.
    # Guard it -- otherwise dropping those flags would silently estimate distortion against a
    # rectified stream and no test would catch it.
    assert np.allclose(res.D, 0.0, atol=1e-9)      # zero distortion stays zero


def test_audit_against_factory_small_delta():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    free = calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="free")
    d = audit_against_factory(free, K_TRUE)
    assert abs(d["d_fx_px"]) < 1.0 and abs(d["d_fy_px"]) < 1.0
    assert abs(d["d_cx_px"]) < 1.0 and abs(d["d_cy_px"]) < 1.0


def test_audit_requires_K0():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    with pytest.raises(ValueError):
        calibrate_intrinsics(dets, board, IMG, K0=None, mode="audit")


def test_free_reports_parameter_std():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    res = calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="free")
    # calibrateCameraExtended path -> per-parameter uncertainties present and finite
    assert {"fx", "fy", "cx", "cy"}.issubset(res.param_std)
    assert all(np.isfinite(v) and v >= 0 for v in res.param_std.values())


def test_rational8_recovers_K_and_has_8_coeffs():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    res = calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="free", distortion_model="rational8")
    assert res.distortion_model == "rational8"
    assert res.rms_px < 0.1
    assert abs(res.K[0, 0] - 700) < 2.0
    # rational model estimates 8 distortion coeffs (k1..k6, p1, p2); OpenCV may pad the vector
    assert res.D.reshape(-1).shape[0] >= 8
    # exercise the n_dist=8 slice of _STD_ORDER: the rational-only radial std-devs must be present
    assert {"k4", "k5", "k6"}.issubset(res.param_std)
    assert all(np.isfinite(v) and v >= 0 for v in res.param_std.values())
    # exercise the n_dist=8 slicing of _STD_ORDER: the rational-only k4/k5/k6 uncertainties must
    # be emitted (a mis-slice / wrong _STD_ORDER would drop or mis-name them) and stay finite & >=0
    assert {"k4", "k5", "k6"}.issubset(res.param_std)
    assert all(np.isfinite(v) and v >= 0 for v in res.param_std.values())


def test_bad_distortion_model_rejected():
    board = make_board(BoardConfig())
    dets = _make_views(board, n=16)
    with pytest.raises(ValueError):
        calibrate_intrinsics(dets, board, IMG, K0=K_TRUE, mode="free", distortion_model="nope")


def test_fisheye_recovers_known_fisheye_K_or_raises_cleanly():
    """fisheye is best-effort on planar ChArUco; it must EITHER recover a sane K OR raise a clear
    RuntimeError (never a bare cv2.error, never a silently-wrong K)."""
    board = make_board(BoardConfig())
    objp = np.asarray(board.getChessboardCorners(), np.float64).reshape(-1, 3)
    N = objp.shape[0]
    cx, cy = objp[:, 0].mean(), objp[:, 1].mean()
    Kf = np.array([[330.0, 0, 640.0], [0, 330.0, 360.0], [0, 0, 1.0]])
    Df = np.array([0.04, -0.005, 0.001, 0.0])

    def fproj(Xc):
        x, y, z = Xc[:, 0], Xc[:, 1], Xc[:, 2]
        a, b = x / z, y / z
        r = np.sqrt(a * a + b * b)
        th = np.arctan(r)
        k1, k2, k3, k4 = Df
        thd = th * (1 + k1 * th**2 + k2 * th**4 + k3 * th**6 + k4 * th**8)
        s = np.where(r > 1e-9, thd / r, 1.0)
        return np.stack([Kf[0, 0] * a * s + Kf[0, 2], Kf[1, 1] * b * s + Kf[1, 2]], axis=1)

    rng = np.random.default_rng(2)
    dets = []
    while len(dets) < 20:
        rv = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-0.3, 0.3)])
        rv = rv / np.linalg.norm(rv) * np.radians(rng.uniform(8, 25))
        R, _ = cv2.Rodrigues(rv)
        Z = rng.uniform(0.6, 1.0)
        t = np.array([-cx, -cy, Z]) + np.array([rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), 0])
        Xc = (R @ objp.T).T + t
        if (Xc[:, 2] <= 0.05).any():
            continue
        pts = fproj(Xc)
        if pts[:, 0].min() < 5 or pts[:, 0].max() > IMG[0] - 5:
            continue
        if pts[:, 1].min() < 5 or pts[:, 1].max() > IMG[1] - 5:
            continue
        dets.append(Detection(charuco_corners=pts.reshape(-1, 1, 2).astype(np.float32),
                              charuco_ids=np.arange(N, dtype=np.int32).reshape(-1, 1),
                              n_corners=N, laplacian_var=120.0))
    try:
        res = calibrate_intrinsics(dets, board, IMG, mode="free", distortion_model="fisheye")
    except RuntimeError:
        return  # acceptable: clear, actionable failure on a planar target
    assert res.distortion_model == "fisheye"
    assert abs(res.K[0, 0] - 330) < 10.0   # recovered the known fisheye focal length
    assert res.D.reshape(-1).shape[0] == 4
