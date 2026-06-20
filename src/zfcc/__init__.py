"""zfcc -- ZED 2i <-> Flexiv Rizon ChArUco hand-eye calibration.

Strict, eye-to-hand (fixed camera, board on the flange) calibration that recovers the metric
``T_base_camera`` together with a falsifiable validation suite (pose-diversity gates, five-solver
cross-check, AX=XB residuals, leave-one-out stability, and a physical touch test).

The math core (``se3``, ``handeye``, ``diversity``, ``validate``) is pure-numpy / lazy-cv2 and fully
tested hardware-free; the hardware shims (``zed_io``, ``robot_io``) use guarded imports so neither the
ZED SDK nor flexivrdk is needed to import the package, run the tests, or solve from a saved session.
"""
from __future__ import annotations

from . import se3  # noqa: F401  (pure-numpy core, safe to import eagerly)

__version__ = "0.2.0"

__all__ = ["se3", "__version__"]


def __getattr__(name):
    # Lazy submodule access so `import zfcc` never pulls cv2/pyzed/flexivrdk transitively.
    import importlib

    if name in {"board", "config", "coverage", "detect", "diversity", "handeye", "intrinsics",
                "refine", "robot_io", "session", "touch_test", "validate", "yaml_out", "zed_io"}:
        return importlib.import_module(f".{name}", __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
