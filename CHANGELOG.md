# Changelog

All notable changes to this project are documented here. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); the project uses semantic versioning.

## [0.2.0] - 2026-06-19

Rigor + documentation pass, driven by a verified survey of established calibration tooling (OpenCV,
calib.io, MoveIt Calibration, easy_handeye, industrial_calibration, Kalibr, multical, ROS
camera_calibration) — see `docs/PRIOR_ART.md`.

### Added
- **`docs/TUTORIAL.md`** — detailed first-timer end-to-end walkthrough (board prep → capture → solve
  → validate → touch test) with verified numbers, the four day-one mistakes, and honest caveats on
  standards vs repo-chosen bars.
- **`docs/PRIOR_ART.md`** — comparison of the major tools and the rationale for our choices.
- **Distinct-working-distance gate**: `diversity` now enforces `min_distinct_depths` (was declared in
  config but never wired in) from the camera-frame board depths, refusing single-depth (weak-Z) sets.
- **`coverage` module**: aggregate intrinsics-coverage report + gate (ROS `camera_calibration`
  X/Y/Size/Skew concept) with a corner-density heatmap PNG; wired into `zfcc-intrinsics`.
- **`intrinsics` distortion-model selector**: `brown5` (default), `rational8` (`CALIB_RATIONAL_MODEL`,
  wide lenses), and best-effort `fisheye`; switched to `cv2.calibrateCameraExtended` to report
  **per-parameter standard deviations** (uncertainties). `--distortion-model` on `zfcc-intrinsics`.
- **`refine` module** (optional, `pip install '.[refine]'`): scipy nonlinear refinement of the
  extrinsic seeded by the DANIILIDIS solution, reporting covariance (the industrial_calibration idea).

### Fixed
- Corrected the "calibrateCameraCharuco was removed in 4.7" wording across docs/code: it was *moved
  into objdetect and deprecated*; the common `AttributeError` is an `opencv-python` +
  `opencv-contrib-python` install conflict (now documented as a day-one fix).
- Removed the dead `if False` rational-model branch in `intrinsics`.
- `fisheye` path converts OpenCV's cryptic planar-target assertion into a clear, actionable error.

### Gate honesty (driven by a 50-agent adversarial review of the above)
- Made the verdict **match the docs**: `AX=XB` max residual and leave-one-out origin std now escalate
  to **FAIL** (were silently WARN despite `*_fail` names); added the cross-solver rotation WARN bar
  and a robot-world cross-check WARN bar; `_grade` now records `verdict_reasons` and no longer
  early-returns (strictest verdict wins, every failing bar reported).
- Reframed the docs to be exact about what gates the write vs what is advisory / a separate tool /
  a **manual** post-write check (the touch test and intrinsics RMS are NOT in the solve verdict);
  wired the `intrinsics_rms_*` bars into the `zfcc-intrinsics` tool; removed the unused
  `per_view_rms_multiple` constant.
- `distinct_depths` now counts *relative* separation (greedy merge) instead of phase-dependent
  absolute bins; `refine` reports `covariance_available` instead of silently emitting an empty std
  dict on a singular Jacobian; corrected the Rizon 4s repeatability figure (~0.05 mm, ISO 9283).
- Strengthened the test suite (62 → 81 tests): non-tautological coplanarity/refine tests, isolated
  coverage size/skew gates, audit-mode zero-distortion assertion, rational8 std-dev keys.

## [0.1.0] - 2026-06-19

Initial public release.

### Added
- Eye-to-hand ChArUco calibration for a **fixed ZED 2i** observing a board on the **Flexiv Rizon
  flange**, recovering the metric `T_base_camera`.
- `se3` pure-numpy SE(3) core (the single place rigid-transform inversion lives), scalar-first
  quaternions matching Flexiv/RDK.
- Five hand-eye solvers (TSAI, PARK, HORAUD, ANDREFF, DANIILIDIS) with DANIILIDIS as primary, plus an
  independent `calibrateRobotWorldHandEye` cross-check.
- **ChArUco intrinsics** (`fx, fy, cx, cy`, distortion) with an **audit mode** that confirms the
  rectified ZED factory K without overwriting it, and a free-solve mode.
- **Pose-diversity / degeneracy gates** that refuse a near-coplanar pose set (the failure mode of the
  old 8-coplanar-marker calibration).
- Validation suite: cross-solver spread, AX=XB residuals, leave-one-out stability, base-frame corner
  error, and a physical **touch test**.
- Drop-in `T_base_zed2i.yaml` writer matching ActAhead's loader schema.
- Guarded hardware shims (`zed_io`, `robot_io`) so the package installs, tests, and solves offline
  without the ZED SDK or flexivrdk.
- Synthetic, hardware-free test suite (round-trips a known extrinsic to prove the inversion sign) and
  GitHub Actions CI on Python 3.9/3.11/3.12.
