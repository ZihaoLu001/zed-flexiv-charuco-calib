"""Camera-intrinsics calibration from ChArUco views.

YES -- a ChArUco board recovers the full pinhole model (fx, fy, cx, cy + distortion), exactly like a
plain chessboard but robust to partial/occluded views because each detected corner carries its ID.
The modern path (OpenCV >= 4.7) is:
    board.matchImagePoints(corners, ids) -> (objpoints, imgpoints) per view
    cv2.calibrateCameraExtended(objpoints, imgpoints, image_size, K0, D0, flags=...)

``cv2.aruco.calibrateCameraCharuco`` was not cleanly deleted -- it was MOVED from contrib into the
``objdetect`` module and DEPRECATED in 4.7 in favour of ``board.matchImagePoints`` + ``cv2.solvePnP`` /
``cv2.calibrateCamera``. The ``AttributeError`` people hit on 4.7 is usually an install conflict
(``opencv-python`` and ``opencv-contrib-python`` co-installed). We use the modern path exclusively.

Distortion model (free mode): start simple. ``brown5`` (k1,k2,p1,p2,k3) is the default and right for
normal lenses; ``rational8`` (CALIB_RATIONAL_MODEL: k1..k6 + p1,p2) only for wide / strong-radial
lenses (it overfits otherwise). Both go through the robust ``cv2.calibrateCameraExtended`` path.
``fisheye`` (the separate equidistant model, k1..k4) is BEST-EFFORT only: OpenCV's
``cv2.fisheye.calibrate`` extrinsic initialization is documented to fail data-dependently on PLANAR
targets like a ChArUco board, so this branch raises a clear, actionable RuntimeError instead of a
cryptic assertion when it cannot initialize. For true fisheye / >~120-150 deg FOV lenses prefer a
dedicated tool (Kalibr) or a non-planar target; ``rational8`` covers most non-fisheye wide lenses.

IMPORTANT for the ZED 2i: its SDK left stream is already RECTIFIED (D == 0) against a factory-
calibrated K. We therefore run intrinsics in AUDIT mode by default: seed K0 with the factory matrix,
FIX principal point + focal length + zero distortion, and only confirm the reprojection RMS is small
and that a free solve lands close to factory. We do NOT overwrite the ZED's K -- the depth stream is
computed against the factory model, so a hand-rolled K would desync RGB and depth, and a wide-FOV /
fisheye model on an already-rectified stream would model nothing real. The free solve is reported for
transparency only. The distortion_model selector matters only for RAW / non-ZED streams.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = ["IntrinsicsResult", "DISTORTION_MODELS", "calibrate_intrinsics", "audit_against_factory"]

DISTORTION_MODELS = ("brown5", "rational8", "fisheye")

# stdDeviationsIntrinsics order returned by cv2.calibrateCameraExtended
_STD_ORDER = ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
              "s1", "s2", "s3", "s4", "taux", "tauy"]


@dataclass
class IntrinsicsResult:
    K: np.ndarray
    D: np.ndarray
    rms_px: float
    image_size: tuple[int, int]
    per_view_rms_px: list
    n_views: int
    mode: str                              # "audit" (fixed) or "free"
    distortion_model: str = "brown5"
    param_std: dict = field(default_factory=dict)   # parameter -> standard deviation (uncertainty)

    def as_dict(self) -> dict:
        return {
            "K": self.K.tolist(),
            "D": self.D.reshape(-1).tolist(),
            "rms_px": round(self.rms_px, 4),
            "image_size": list(self.image_size),
            "n_views": self.n_views,
            "mode": self.mode,
            "distortion_model": self.distortion_model,
            "param_std": {k: round(float(v), 6) for k, v in self.param_std.items()},
            "per_view_rms_px": [round(float(v), 4) for v in self.per_view_rms_px],
        }


def _collect_points(detections, board, min_corners):
    objpoints, imgpoints = [], []
    for det in detections:
        if det.charuco_ids is None or det.n_corners < min_corners:
            continue
        objp, imgp = board.matchImagePoints(det.charuco_corners, det.charuco_ids)
        if objp is None or len(objp) < min_corners:
            continue
        objpoints.append(np.asarray(objp, dtype=np.float32).reshape(-1, 1, 3))
        imgpoints.append(np.asarray(imgp, dtype=np.float32).reshape(-1, 1, 2))
    return objpoints, imgpoints


def _std_dict(std_intrinsics, n_dist):
    if std_intrinsics is None:
        return {}
    s = np.asarray(std_intrinsics, dtype=float).reshape(-1)
    names = _STD_ORDER[:4] + _STD_ORDER[4:4 + n_dist]
    return {names[i]: float(s[i]) for i in range(min(len(names), len(s)))}


def _fisheye_calibrate(objpoints, imgpoints, image_size):
    import cv2

    # cv2.fisheye.calibrate is shape-picky: it needs float64 points shaped (1, N, 3) / (1, N, 2) per
    # view (NOT the (N,1,*) layout calibrateCamera accepts) or it throws InitExtrinsics assertions.
    obj = [o.reshape(1, -1, 3).astype(np.float64) for o in objpoints]
    img = [p.reshape(1, -1, 2).astype(np.float64) for p in imgpoints]
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    # NB: NO CALIB_CHECK_COND -- it rejects "ill-conditioned" planar views, and a ChArUco board IS a
    # planar target, so CHECK_COND throws on essentially every ChArUco fisheye dataset. We keep
    # RECOMPUTE_EXTRINSIC + FIX_SKEW (the robust pair for planar targets).
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    try:
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(obj, img, tuple(image_size), K, D, flags=flags)
    except cv2.error as e:
        # cv2.fisheye.calibrate's extrinsic init (InitExtrinsics / CHECK_COND) is known to fail
        # data-dependently on PLANAR targets like a ChArUco board. Surface an actionable error
        # instead of a cryptic OpenCV assertion. brown5/rational8 cover most non-fisheye wide lenses.
        raise RuntimeError(
            "cv2.fisheye.calibrate failed on this ChArUco dataset (OpenCV's fisheye extrinsic "
            "initialization is fragile on planar targets): " + str(e).splitlines()[-1] +
            "  --  For true fisheye / >~120-150 deg FOV lenses prefer a dedicated tool (Kalibr) or a "
            "non-planar target; for normal/moderately-wide lenses use distortion_model='rational8'."
        ) from e
    per_view = []
    for o, p, rv, tv in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.fisheye.projectPoints(o, rv, tv, K, D)
        e = proj.reshape(-1, 2) - p.reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))))
    return float(rms), np.asarray(K), np.asarray(D), per_view


def calibrate_intrinsics(detections, board, image_size, K0=None, mode="audit",
                         distortion_model="brown5", min_corners: int = 8) -> IntrinsicsResult:
    """Solve (or audit) intrinsics from a list of Detection objects.

    mode="audit": requires K0 (factory); fixes focal length + principal point + zero distortion, so
                  the only free parameters are the per-view extrinsics -> the returned RMS is the
                  reprojection error of the FACTORY model on real data (the number that matters).
                  distortion_model is ignored in audit mode.
    mode="free":  full solve. distortion_model in {brown5, rational8, fisheye}; brown5/rational8 use
                  calibrateCameraExtended (so we also report per-parameter std deviations); fisheye
                  routes to cv2.fisheye.calibrate (equidistant model, no std deviations available).
    """
    import cv2

    if distortion_model not in DISTORTION_MODELS:
        raise ValueError(f"distortion_model must be one of {DISTORTION_MODELS}, got {distortion_model!r}")

    objpoints, imgpoints = _collect_points(detections, board, min_corners)
    if len(objpoints) < 4:               # OpenCV's own sample gate: need >= 4 valid views
        raise ValueError(f"need >=4 usable views for intrinsics, got {len(objpoints)}")
    w, h = int(image_size[0]), int(image_size[1])

    if mode == "free" and distortion_model == "fisheye":
        rms, K, D, per_view = _fisheye_calibrate(objpoints, imgpoints, (w, h))
        return IntrinsicsResult(K=K, D=D, rms_px=rms, image_size=(w, h), per_view_rms_px=per_view,
                                n_views=len(objpoints), mode=mode, distortion_model="fisheye",
                                param_std={})

    if mode == "audit":
        if K0 is None:
            raise ValueError("audit mode requires the factory K0")
        K_init = np.asarray(K0, dtype=float).copy()
        D_init = np.zeros((5, 1), dtype=float)
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT
                 | cv2.CALIB_FIX_FOCAL_LENGTH | cv2.CALIB_ZERO_TANGENT_DIST
                 | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
        n_dist = 5
        dmodel = "brown5"
    else:  # free
        flags = 0
        if distortion_model == "rational8":
            flags |= cv2.CALIB_RATIONAL_MODEL
            n_dist = 8
        else:  # brown5
            n_dist = 5
        K_init = None
        D_init = None
        if K0 is not None:
            flags |= cv2.CALIB_USE_INTRINSIC_GUESS
            K_init = np.asarray(K0, dtype=float).copy()
        dmodel = distortion_model

    rms, K, D, _rvecs, _tvecs, std_int, _std_ext, per_view = cv2.calibrateCameraExtended(
        objpoints, imgpoints, (w, h), K_init, D_init, flags=flags)

    return IntrinsicsResult(
        K=np.asarray(K), D=np.asarray(D), rms_px=float(rms), image_size=(w, h),
        per_view_rms_px=[float(v) for v in np.asarray(per_view).reshape(-1)],
        n_views=len(objpoints), mode=mode, distortion_model=dmodel,
        param_std=_std_dict(std_int, n_dist))


def audit_against_factory(free: IntrinsicsResult, factory_K) -> dict:
    """Compare a FREE intrinsic solve to the factory K -- a sanity bound on the rectified stream."""
    Kf = np.asarray(factory_K, dtype=float)
    Kc = np.asarray(free.K, dtype=float)
    return {
        "d_fx_px": float(Kc[0, 0] - Kf[0, 0]),
        "d_fy_px": float(Kc[1, 1] - Kf[1, 1]),
        "d_cx_px": float(Kc[0, 2] - Kf[0, 2]),
        "d_cy_px": float(Kc[1, 2] - Kf[1, 2]),
        "free_rms_px": round(free.rms_px, 4),
    }
