"""Aggregate intrinsics-coverage assessment (the ROS camera_calibration X/Y/Size/Skew concept).

A low reprojection RMS does NOT prove a good intrinsics calibration -- it only shows the model can fit
the data (calib.io). The biggest preventable failure is POOR COVERAGE: if the board never reaches the
image edges, never varies its working distance, and is never tilted, then focal length and the
principal point are weakly observable and the solve can be confidently wrong. This module aggregates
the per-view detections into four fill metrics and gates on them BEFORE trusting an intrinsics solve,
and can render a corner-density heatmap PNG for inspection.

  X / Y   : how far the board centroid roams across the image width / height (fraction)
  Size    : ratio of the largest to smallest per-view board bounding-box area (near vs far)
  Skew    : the maximum board tilt across views (needs board poses; foreshortening reveals f, cx, cy)
  Area    : fraction of a grid of image cells that received at least one detected corner (any view)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import CoverageGates

__all__ = ["CoverageReport", "coverage_report", "render_coverage_heatmap"]


@dataclass
class CoverageReport:
    n_views: int
    x_center_fill: float
    y_center_fill: float
    corner_area_fill: float
    size_ratio: float
    max_skew_deg: float          # -1 if board poses were not supplied
    grid_counts: np.ndarray      # (grid_y, grid_x) corner-count heatmap
    verdict: str                 # PASS / WARN / FAIL
    reasons: list = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "n_views": self.n_views,
            "x_center_fill": round(self.x_center_fill, 3),
            "y_center_fill": round(self.y_center_fill, 3),
            "corner_area_fill": round(self.corner_area_fill, 3),
            "size_ratio": round(self.size_ratio, 3),
            "max_skew_deg": round(self.max_skew_deg, 2),
            "verdict": self.verdict,
            "reasons": self.reasons,
        }


def _corners_xy(det):
    if det.charuco_corners is None or det.n_corners == 0:
        return None
    return np.asarray(det.charuco_corners, dtype=float).reshape(-1, 2)


def coverage_report(detections, image_size, board_poses_T=None,
                    gates: CoverageGates | None = None) -> CoverageReport:
    """Aggregate per-view detections into coverage metrics + a PASS/WARN/FAIL verdict.

    ``board_poses_T`` (list of T_cam_board, one per *usable* detection) is optional; without it the
    skew gate is skipped (reported as -1). Detections with no corners are ignored.
    """
    g = gates or CoverageGates()
    w, h = int(image_size[0]), int(image_size[1])
    grid = np.zeros((g.grid_y, g.grid_x), dtype=int)
    centroids, areas = [], []
    for det in detections:
        pts = _corners_xy(det)
        if pts is None:
            continue
        centroids.append(pts.mean(axis=0))
        bb = (pts[:, 0].max() - pts[:, 0].min()) * (pts[:, 1].max() - pts[:, 1].min())
        areas.append(bb / float(max(w * h, 1)))
        for x, y in pts:
            cx = min(g.grid_x - 1, max(0, int(x / max(w, 1) * g.grid_x)))
            cy = min(g.grid_y - 1, max(0, int(y / max(h, 1) * g.grid_y)))
            grid[cy, cx] += 1
    n = len(centroids)
    if n == 0:
        return CoverageReport(0, 0, 0, 0, 0, -1.0, grid, "FAIL", ["FAIL: no detections"])

    C = np.asarray(centroids)
    x_fill = float((C[:, 0].max() - C[:, 0].min()) / max(w, 1))
    y_fill = float((C[:, 1].max() - C[:, 1].min()) / max(h, 1))
    area_fill = float(np.count_nonzero(grid) / grid.size)
    a = np.asarray(areas)
    size_ratio = float(a.max() / a.min()) if a.min() > 0 else 0.0

    max_skew = -1.0
    if board_poses_T:
        skews = []
        for T in board_poses_T:
            # tilt = angle between the board normal (board z in camera frame) and the camera axis
            n_cam = np.asarray(T, dtype=float)[:3, 2]
            cos = abs(np.clip(n_cam[2] / (np.linalg.norm(n_cam) + 1e-12), -1, 1))
            skews.append(float(np.degrees(np.arccos(cos))))
        max_skew = float(max(skews)) if skews else -1.0

    reasons, verdict = [], "PASS"

    def warn(msg):
        nonlocal verdict
        if verdict != "FAIL":
            verdict = "WARN"
        reasons.append("WARN: " + msg)

    if x_fill < g.min_x_center_fill:
        warn(f"board spans only {x_fill:.0%} of width (< {g.min_x_center_fill:.0%}); slide it left/right")
    if y_fill < g.min_y_center_fill:
        warn(f"board spans only {y_fill:.0%} of height (< {g.min_y_center_fill:.0%}); slide it up/down")
    if area_fill < g.min_corner_area_fill:
        warn(f"only {area_fill:.0%} of cells saw a corner (< {g.min_corner_area_fill:.0%}); reach the edges")
    if size_ratio < g.min_size_ratio:
        warn(f"board size ratio {size_ratio:.2f} (< {g.min_size_ratio}); vary working distance (near+far)")
    if max_skew >= 0 and max_skew < g.min_skew_deg:
        warn(f"max tilt {max_skew:.0f} deg (< {g.min_skew_deg:.0f}); tilt the board (reveals focal length)")

    return CoverageReport(n_views=n, x_center_fill=x_fill, y_center_fill=y_fill,
                          corner_area_fill=area_fill, size_ratio=size_ratio, max_skew_deg=max_skew,
                          grid_counts=grid, verdict=verdict, reasons=reasons)


def render_coverage_heatmap(report: CoverageReport, out_path: str, cell_px: int = 60) -> str:
    """Render the corner-density grid to a color heatmap PNG for visual inspection."""
    import cv2

    gy, gx = report.grid_counts.shape
    g = report.grid_counts.astype(float)
    norm = (g / g.max() * 255).astype(np.uint8) if g.max() > 0 else g.astype(np.uint8)
    big = cv2.resize(norm, (gx * cell_px, gy * cell_px), interpolation=cv2.INTER_NEAREST)
    heat = cv2.applyColorMap(big, cv2.COLORMAP_JET)
    cv2.imwrite(out_path, heat)
    return out_path
