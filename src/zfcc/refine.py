"""Optional nonlinear (bundle-adjustment-style) refinement of the eye-to-hand result.

The five closed-form AX=XB solvers are noise-sensitive; the rigorous accuracy ceiling
(industrial_calibration / Ceres) is a nonlinear least-squares refinement seeded by the closed-form
solution, which also yields parameter covariance. This module refines ``T_base_camera`` and
``T_flange_board`` JOINTLY by minimizing the pose-consistency residual

    r_i = pose_error( T_base_camera @ T_cam_board_i ,  T_base_flange_i @ T_flange_board )

over all poses (the same data the closed-form solver consumed -- no raw image points needed). It uses
``scipy.optimize.least_squares`` (a guarded, optional dependency: ``pip install '.[refine]'``); the
rest of the package never imports scipy.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import se3

__all__ = ["RefineResult", "refine_extrinsic"]


def _cv():
    import cv2
    return cv2


def _T_from_params(p):
    cv2 = _cv()
    R, _ = cv2.Rodrigues(np.asarray(p[:3], dtype=float).reshape(3, 1))
    return se3.Rt_to_T(R, p[3:6])


def _params_from_T(T):
    cv2 = _cv()
    rvec, _ = cv2.Rodrigues(np.asarray(T, dtype=float)[:3, :3])
    return np.concatenate([rvec.reshape(3), np.asarray(T, dtype=float)[:3, 3]])


@dataclass
class RefineResult:
    T_base_camera: np.ndarray
    T_flange_board: np.ndarray
    rms_translation_mm: float
    rms_rotation_deg: float
    cost: float
    n_poses: int
    converged: bool
    param_std: dict          # std-dev of the 6 T_base_camera params (rx,ry,rz [rad], tx,ty,tz [m])
    delta_from_init_mm: float
    covariance_available: bool = False   # False when the Jacobian was singular (e.g. zero-residual)

    def as_dict(self) -> dict:
        return {
            "T_base_camera": self.T_base_camera.tolist(),
            "rms_translation_mm": round(self.rms_translation_mm, 4),
            "rms_rotation_deg": round(self.rms_rotation_deg, 4),
            "cost": float(self.cost),
            "n_poses": self.n_poses,
            "converged": self.converged,
            "covariance_available": self.covariance_available,
            "delta_from_init_mm": round(self.delta_from_init_mm, 4),
            "param_std": {k: round(float(v), 6) for k, v in self.param_std.items()},
        }


def refine_extrinsic(T_base_camera_init, T_flange_board_init, flange_poses_T, board_poses_T,
                     rot_weight_m: float = 0.25) -> RefineResult:
    """Refine (T_base_camera, T_flange_board) by nonlinear least-squares on the pose residual.

    rot_weight_m scales the rotation residual (radians) into metres so translation and rotation are
    comparably weighted (~a nominal lever arm); default 0.25 m. Seed with the closed-form solution.
    """
    try:
        from scipy.optimize import least_squares
    except Exception as e:  # pragma: no cover - optional dep
        raise RuntimeError("scipy is required for refine_extrinsic; install with: pip install "
                           "'.[refine]'") from e

    flange = [np.asarray(T, dtype=float) for T in flange_poses_T]
    boards = [np.asarray(T, dtype=float) for T in board_poses_T]
    if len(flange) != len(boards) or len(flange) < 3:
        raise ValueError("need >=3 paired poses to refine")

    x0 = np.concatenate([_params_from_T(T_base_camera_init), _params_from_T(T_flange_board_init)])

    def residuals(x):
        T_bc = _T_from_params(x[:6])
        T_fb = _T_from_params(x[6:12])
        cv2 = _cv()
        out = []
        for T_bf, T_cb in zip(flange, boards):
            A = T_bc @ T_cb
            B = T_bf @ T_fb
            et = A[:3, 3] - B[:3, 3]
            rvec, _ = cv2.Rodrigues(A[:3, :3] @ B[:3, :3].T)
            out.append(np.concatenate([et, rvec.reshape(3) * rot_weight_m]))
        return np.concatenate(out)

    sol = least_squares(residuals, x0, method="lm")
    T_bc = _T_from_params(sol.x[:6])
    T_fb = _T_from_params(sol.x[6:12])

    # report residuals in physical units
    trans_mm, rot_deg = [], []
    for T_bf, T_cb in zip(flange, boards):
        A = T_bc @ T_cb
        B = T_bf @ T_fb
        trans_mm.append(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0)
        rot_deg.append(se3.rotation_angle_deg(A[:3, :3] @ B[:3, :3].T))
    rms_t = float(np.sqrt(np.mean(np.square(trans_mm))))
    rms_r = float(np.sqrt(np.mean(np.square(rot_deg))))

    # covariance from the Gauss-Newton approximation: cov = sigma^2 * (J^T J)^-1. On a (near-)
    # zero-residual fit J^T J is singular/ill-conditioned, so covariance is genuinely unavailable;
    # we report covariance_available=False rather than silently emitting an empty dict.
    param_std = {}
    covariance_available = False
    try:
        J = sol.jac
        m, nparams = J.shape
        dof = max(m - nparams, 1)
        sigma2 = 2.0 * sol.cost / dof
        cov = np.linalg.inv(J.T @ J) * sigma2
        names = ["rx", "ry", "rz", "tx", "ty", "tz"]   # T_base_camera block (first 6)
        stds = {nm: float(np.sqrt(max(cov[i, i], 0.0))) for i, nm in enumerate(names)}
        if all(np.isfinite(v) for v in stds.values()):
            param_std = stds
            covariance_available = True
    except Exception:
        param_std = {}
        covariance_available = False

    delta = float(np.linalg.norm(T_bc[:3, 3] - np.asarray(T_base_camera_init)[:3, 3]) * 1000.0)
    return RefineResult(T_base_camera=T_bc, T_flange_board=T_fb, rms_translation_mm=rms_t,
                        rms_rotation_deg=rms_r, cost=float(sol.cost), n_poses=len(flange),
                        converged=bool(sol.success), param_std=param_std, delta_from_init_mm=delta,
                        covariance_available=covariance_available)
