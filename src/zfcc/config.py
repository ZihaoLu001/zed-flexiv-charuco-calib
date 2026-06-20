"""Typed configuration dataclasses, loadable from the YAML files in ``configs/``."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path

import yaml

__all__ = [
    "BoardConfig",
    "ZedConfig",
    "RobotConfig",
    "DiversityGates",
    "CoverageGates",
    "PassBars",
    "load_yaml",
]


def load_yaml(path: str | Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _from_dict(cls, d: dict):
    """Build a dataclass from a dict, ignoring unknown keys (forward-compatible)."""
    fields = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in (d or {}).items() if k in fields})


@dataclass
class BoardConfig:
    """A Calib.io ChArUco board. NOTE: ``squares_xy`` is (cols=squaresX, rows=squaresY).

    A board catalogued as "9x14" (9 rows x 14 cols) -> squares_xy = [14, 9]."""
    squares_xy: tuple[int, int] = (14, 9)
    square_length_m: float = 0.040
    marker_length_m: float = 0.030
    aruco_dict: str = "DICT_5X5_100"
    legacy_pattern: bool | None = None  # None = resolve empirically on first capture

    @classmethod
    def load(cls, path: str | Path) -> BoardConfig:
        d = load_yaml(path)
        d = d.get("board", d)
        if "squares_xy" in d:
            d["squares_xy"] = tuple(int(v) for v in d["squares_xy"])
        return _from_dict(cls, d)


@dataclass
class ZedConfig:
    resolution: str = "HD720"
    depth_mode: str = "NEURAL"          # ULTRA is deprecated in SDK 5.x
    disable_self_calib: bool = True     # freeze K across opens for reproducibility
    depth_minimum_distance_m: float = 0.2
    serial: str | None = None

    @classmethod
    def load(cls, path: str | Path) -> ZedConfig:
        d = load_yaml(path)
        return _from_dict(cls, d.get("zed", d))


@dataclass
class RobotConfig:
    host: str = "192.168.0.105"
    serial: str | None = None
    owner: str = "handeye"
    max_joint_speed: float = 0.3
    settle_vel_eps: float = 1e-3
    settle_dwell_s: float = 1.2
    joint_targets: list[list[float]] = field(default_factory=list)

    @classmethod
    def load(cls, path: str | Path) -> RobotConfig:
        d = load_yaml(path)
        return _from_dict(cls, d.get("robot", d))


@dataclass
class DiversityGates:
    min_poses: int = 15
    min_poses_pass: int = 20
    min_rotation_axes: int = 3
    min_interpose_rotation_deg_warn: float = 30.0
    min_interpose_rotation_deg_pass: float = 60.0
    coplanarity_index_fail: float = 0.04   # smallest/largest singular-value ratio of centred positions
    min_distinct_depths: int = 3           # distinct board-to-camera depths (binned by depth_bin_m)
    depth_bin_m: float = 0.05              # depth resolution for the distinct-depth count (5 cm bins)


@dataclass
class CoverageGates:
    """Aggregate intrinsics-coverage bars (ROS camera_calibration X/Y/Size/Skew concept).

    Poor coverage makes focal length + principal point unobservable -- the highest-value preventable
    intrinsics failure (calib.io). These gate the board's travel across the frame, its working-distance
    variety, and that at least some views are tilted (foreshortening is what reveals f and the centre)."""
    grid_x: int = 8
    grid_y: int = 6
    min_x_center_fill: float = 0.5    # board centroid must roam >= 50% of image width across views
    min_y_center_fill: float = 0.5    # ... and >= 50% of height
    min_corner_area_fill: float = 0.6 # >= 60% of grid cells must receive a detected corner (any view)
    min_size_ratio: float = 1.5       # max/min per-view board bbox area >= 1.5 (near+far variety)
    min_skew_deg: float = 20.0        # at least one view tilted >= 20 deg (needs board poses)


@dataclass
class PassBars:
    """Every numeric acceptance bar in one place (mirrors configs/pass_bars.yaml).

    Which bars actually gate the written calibration (consumed by ``session._grade``): the
    cross-solver, AX=XB, leave-one-out and robot-world bars below. ``intrinsics_rms_*`` gates the
    SEPARATE ``zfcc-intrinsics`` audit tool. ``touch_test_*`` is operator-reference only (a manual
    post-write ruler check; nothing in the verdict reads it). ``per_frame_pnp_px_fail`` is the
    per-view outlier-rejection threshold."""
    # intrinsics audit (separate tool, not the extrinsic verdict)
    intrinsics_rms_px_pass: float = 0.3
    intrinsics_rms_px_fail: float = 1.0
    # extrinsic write-gates (consumed by session._grade)
    cross_solver_translation_mm_pass: float = 2.0   # > pass -> WARN
    cross_solver_translation_mm_fail: float = 3.0   # > fail -> FAIL
    cross_solver_rotation_deg_pass: float = 0.2     # > pass -> WARN
    cross_solver_rotation_deg_fail: float = 0.5     # > fail -> FAIL
    axxb_translation_mm_fail: float = 2.0           # > fail -> FAIL (pose self-consistency)
    loo_translation_std_mm_fail: float = 5.0        # > fail -> FAIL (leave-one-out instability)
    robot_world_translation_mm_warn: float = 2.0    # robot-world cross-check disagreement -> WARN
    per_frame_pnp_px_fail: float = 1.0              # drop a view whose PnP RMS exceeds this
    # operator reference only -- NOT read by the verdict (manual ruler check after the YAML is written)
    touch_test_mm_pass: float = 3.0
    touch_test_mm_fail: float = 5.0

    @classmethod
    def load(cls, path: str | Path) -> PassBars:
        d = load_yaml(path)
        return _from_dict(cls, d.get("pass_bars", d))
