# Prior art — how others do ChArUco intrinsics + hand-eye, and where this repo sits

This repo did not invent a method; it packages the established OpenCV calibration core with a strict,
falsifiable validation suite tuned for one eye-to-hand setup (fixed ZED 2i, board on a Flexiv flange).
This page situates it against the tools practitioners actually use, so our design choices are explicit
rather than reverse-engineered. All claims here were web-verified against primary sources (links).

## Comparison

| Project | Intrinsics | Extrinsic (hand-eye) | Validation | Notable |
|---|---|---|---|---|
| **OpenCV** `calib3d`/`objdetect` ([src](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)) | `matchImagePoints` → `calibrateCamera(Extended)`; default 5-coef Brown | 5 AX=XB solvers (TSAI/PARK/HORAUD/ANDREFF/**DANIILIDIS**); robot-world AX=ZB (SHAH/LI). Eye-to-hand = invert each pose | RMS + (Extended) per-view RMS & param std-devs; no hand-eye validation | The numerical core every other tool wraps. We use the modern `matchImagePoints` path + all 5 solvers |
| **calib.io** KB ([best practices](https://calib.io/blogs/knowledge-base/calibration-best-practices)) | Guidance: ≥6 imgs, ≥50% area, ±45° tilt, fixed focus | n/a | Residuals must be **structureless**; *low RMS ≠ good calibration*; use param uncertainties | Source of our ±45°/coverage/uncertainty rules |
| **easy_handeye / easy_handeye2** ([repo](https://github.com/marcoesposito1988/easy_handeye2)) | out of scope (uses a pre-calibrated camera + tracker) | OpenCV `calibrateHandEye`; eye-in-hand + eye-on-base; YAML → static TF | "move robot, marker stays fixed" visual check | Best operational **pose-sampling recipe** (rotate ≤90°/axis both ways) — we cite it in the tutorial |
| **MoveIt Calibration** ([repo](https://github.com/moveit/moveit_calibration)) | assumes calibrated camera; detects ChArUco | RViz plugin; **DANIILIDIS default**, Park/Tsai selectable; eye-in-hand + eye-to-hand | guidance: min 5 samples, plateau 12–15, **≥2 rotation axes** | Confirms DANIILIDIS as the sensible default (= our primary) and the sample-count guidance |
| **industrial_calibration** (ROS-I) ([repo](https://github.com/ros-industrial/industrial_calibration)) | joint intrinsics+extrinsics | **Ceres bundle adjustment** (reprojection cost), not closed-form; can also fit robot kinematics | strongest: Ceres cost, per-obs residuals, **covariance** | The accuracy ceiling. Our optional `refine.py` is the lightweight scipy analogue (+ covariance) |
| **Kalibr** ([repo](https://github.com/ethz-asl/kalibr)) | full intrinsics incl. **fisheye/wide-FOV** + rolling shutter | multi-camera & camera-IMU (continuous-time BA) — **not** robot AX=XB | reprojection reports, residual plots | Cite for fisheye/wide-FOV and AprilGrid; **not** a robot hand-eye tool (don't use it for base←camera) |
| **multical** ([repo](https://github.com/oliver-batchelor/multical)) | ChArUco/AprilGrid → bundle-adjusted intrinsics | global BA over a rig + boards; hand-eye for non-overlapping views | reprojection; **holdout** validation (`--fix_intrinsic` re-run) | Clean intrinsic/extrinsic separation + holdout pattern worth emulating |
| **ROS `camera_calibration`** ([index](https://index.ros.org/p/camera_calibration/)) | OpenCV `calibrateCamera`; live **X/Y/Size/Skew** coverage bars | stereo only (not robot hand-eye) | aims < 1 px; rectified-image inspection | The model for our `coverage.py` aggregate gate |
| **JonesCVBS/HandEyeCalibration** ([repo](https://github.com/JonesCVBS/HandEyeCalibration-using-OpenCV)) | chessboard | `calibrateHandEye` across **all** methods, writes one file each | compare the spread across methods | The "run all solvers, compare spread" pattern — which we make a gate |

## Why this repo's choices

- **Closed-form AX=XB (DANIILIDIS primary), with an optional BA refinement.** DANIILIDIS (dual
  quaternion, simultaneous R+t) is MoveIt's default and the most rotation-robust. We additionally run
  the other four solvers and *gate on their agreement* (a data-quality signal), and cross-check with
  the structurally-different `calibrateRobotWorldHandEye`. The reprojection-BA accuracy ceiling
  (industrial_calibration/Ceres) is available as an **optional** scipy refinement in
  [`refine.py`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/refine.py), reporting covariance — off by default.
- **Per-pose inversion for eye-to-hand.** `calibrateHandEye` has no eye-to-hand flag; we invert each
  `T_base_flange` before the call so the output is `T_base_camera`. Inverting only the *final* output
  is the common wrong shortcut — a synthetic test asserts our sign is right.
- **Audit-only intrinsics on the ZED.** The ZED's left stream is rectified against a factory `K` the
  depth engine uses; we confirm it rather than overwrite it. A free solve (`brown5`/`rational8`, with
  parameter std-devs via `calibrateCameraExtended`) is reported for transparency. A best-effort
  `fisheye` path exists for raw wide lenses but is honest about OpenCV's planar-target fragility.
- **Diversity gates stricter than the norm.** MoveIt suggests ≥2 rotation axes and 12–15 samples; we
  FAIL below 15 poses / 3 rotation axes / 3 distinct working distances and on a near-coplanar set —
  precisely the degeneracy that produced our earlier table-marker calibration's ~1–2 cm depth error.
- **A physical touch test as the headline gate.** Reprojection only exercises the camera↔board leg;
  the touch test exercises the whole `base ← camera ← board → tool` chain — the number that predicts
  grasp success.

## What we deliberately did *not* build

- A full Ceres reprojection bundle adjustment (industrial_calibration already does this well; our
  scipy refinement is the lightweight, dependency-light version).
- A fisheye/omnidirectional intrinsics pipeline (use Kalibr — OpenCV's planar-target fisheye solver is
  unreliable, and the ZED stream is rectified anyway).
- Multi-camera rig / camera-IMU calibration (out of scope; that's Kalibr/multical territory).

## Sources

OpenCV: [ChArUco calibration tutorial](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html) ·
[hand-eye source](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) ·
[`calibrateCameraCharuco` AttributeError #23493](https://github.com/opencv/opencv/issues/23493) ·
[4.6 ChArUco parity #23152](https://github.com/opencv/opencv/issues/23152) ·
[detector homography ignores distortion #23873](https://github.com/opencv/opencv/issues/23873) ·
[fisheye module](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html).
calib.io: [best practices](https://calib.io/blogs/knowledge-base/calibration-best-practices) ·
[5 biggest mistakes](https://calib.io/blogs/knowledge-base/5-biggest-calibration-mistakes).
[easy_handeye](https://github.com/IFL-CAMP/easy_handeye) ·
[MoveIt hand-eye tutorial](https://moveit.picknik.ai/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html) ·
[industrial_calibration](https://github.com/ros-industrial/industrial_calibration) ·
[Kalibr](https://github.com/ethz-asl/kalibr) ·
[multical](https://github.com/oliver-batchelor/multical) ·
[ROS camera_calibration](https://index.ros.org/p/camera_calibration/).
