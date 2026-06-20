# zed-flexiv-charuco-calib

**Strict eye-to-hand calibration between a fixed ZED 2i camera and a Flexiv Rizon arm, using a
ChArUco board on the flange — with a falsifiable validation suite.**

[![ci](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/actions/workflows/ci.yml/badge.svg)](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/actions/workflows/ci.yml)
&nbsp;Apache-2.0&nbsp;·&nbsp;Python 3.9–3.12&nbsp;·&nbsp;OpenCV ≥ 4.7 (`opencv-contrib`)

It recovers the fixed rigid transform **`T_base_camera`** (camera pose in the robot base frame) so
that a 3D point the camera sees can be commanded directly in robot coordinates — the number a grasp
pipeline depends on. The output is a drop-in `T_base_zed2i.yaml`.

> 🧭 **First time using a ChArUco board? Start with the step-by-step [TUTORIAL](docs/TUTORIAL.md).**
> It walks you from board prep to a validated calibration and lists the four mistakes first-timers
> hit on day one.

> Built to replace an ad-hoc calibration from **8 coplanar ArUco markers on the table**. That layout
> is *planar*: PnP is accurate in the image plane but weak in depth/tilt, so the recovered camera
> pose can be off by **1–2 cm in Z** while every on-screen check still looks perfect. This repo makes
> that failure mode **impossible to ship silently** — a pose-diversity gate refuses a near-coplanar
> set and a five-solver cross-check must agree before a calibration is written, then a physical touch
> test (a manual operator check, run after the YAML is written) confirms the whole chain on the robot.

---

## Does a ChArUco board also give camera intrinsics? — Yes.

A ChArUco board recovers the **full pinhole model** (`fx, fy, cx, cy` + distortion `k1,k2,p1,p2,k3`),
exactly like a plain chessboard but more robust to partial/occluded views because every detected
corner carries a unique ID. So one board does **both** jobs:

| Quantity | How | Notes |
|---|---|---|
| **Intrinsics** `K`, distortion | `board.matchImagePoints(...)` → `cv2.calibrateCameraExtended(...)` | `cv2.aruco.calibrateCameraCharuco` was moved into `objdetect` & deprecated in 4.7 (an `AttributeError` there usually means `opencv-python` + `opencv-contrib-python` are both installed); this is the current path, and Extended also returns per-parameter uncertainties |
| **Board pose** `T_cam_board` | `board.matchImagePoints(...)` → `cv2.solvePnP(IPPE)` + LM refine | the per-view measurement that feeds hand-eye |
| **Extrinsics** `T_base_camera` | five `cv2.calibrateHandEye` solvers + `calibrateRobotWorldHandEye` cross-check | the goal of this repo |

**Important for the ZED 2i specifically:** the SDK's *left* image is already **rectified** against a
factory-calibrated `K` that the **depth engine also uses**. Re-deriving and overwriting `K` would
desync RGB and depth. So intrinsics here default to **audit mode**: seed the factory `K`, fix focal
length + principal point + zero distortion, and confirm it reprojects the board with sub-pixel RMS;
a free solve is also reported for transparency. Use `--mode free` only if you deliberately want an
independent `K` (e.g. for a non-ZED camera or a raw/unrectified stream).

---

## The method (eye-to-hand), in one paragraph

The camera is **fixed** in the workspace; the ChArUco board is **bolted to the flange**. For each
robot pose we read the **flange** pose `T_base_flange` (from the Flexiv controller) and measure the
board pose `T_cam_board` (from the image). Two unknown rigid transforms are constant across all
poses: the camera in base, `T_base_camera`, and the board on the flange, `T_flange_board`. They obey

```
T_base_camera · T_cam_board  =  T_base_flange · T_flange_board       (for every pose)
```

`cv2.calibrateHandEye` natively solves the *eye-in-hand* problem; for a **fixed camera** you feed the
**inverted** robot poses (`T_gripper_base = inv(T_base_flange)`) and the solver returns
`X = T_base_camera` directly. That single inversion is the whole eye-to-hand trick, and it lives in
exactly one place ([`handeye.solve_eye_to_hand`](src/zfcc/handeye.py)); a synthetic round-trip test
asserts the sign is right (a flipped inversion still returns a clean matrix — just the wrong one).

> This matches your description exactly: *"the ZED 2i estimates the 3D pose of the known-size ChArUco
> board from the image; those board poses are paired with the robot end-effector pose, and hand-eye
> calibration determines the fixed transform between the ZED 2i frame and the robot base frame."* The
> one refinement: pair board poses with the **flange** pose, not the **TCP** pose — the TCP carries
> the gripper tool offset, which would otherwise leak straight into the extrinsic.

See [docs/METHOD.md](docs/METHOD.md) for the full derivation and [docs/FRAMES.md](docs/FRAMES.md) for
frame/quaternion conventions.

---

## Why it is *strict* (the validation suite)

A solver always returns *a* matrix. These checks decide whether to trust it. The **Effect** column is
honest about what each actually does: `FAIL` blocks `zfcc-solve` from writing the YAML, `WARN` writes
but flags, *advisory* is reported, and *manual / separate tool* is not part of the solve verdict at
all. Every gating threshold lives in [`configs/pass_bars.yaml`](configs/pass_bars.yaml).

**Write-gating (the `zfcc-solve` verdict):**

| Check | Bar (default) | Effect |
|---|---|---|
| **Pose-diversity gate** | ≥3 rotation axes, ≥3 distinct working distances, coplanarity index ≥ 0.04, ≥15 poses | **FAIL** |
| **Five-solver agreement** | TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS spread | FAIL > 3 mm / 0.5°; WARN > 2 mm / 0.2° |
| **AX=XB residual** | per-pose self-consistency, max | **FAIL** > 2 mm |
| **Leave-one-out** | a single pose leveraging the fit; origin std | **FAIL** > 5 mm |
| **Robot-world cross-check** | independent `calibrateRobotWorldHandEye` disagreement | WARN > 2 mm |
| **Per-frame PnP RMS** | blurry / grazing views | dropped > 1 px (before solving) |

**Advisory / separate tool / manual (do *not* gate the write):**

| Check | Bar | Where |
|---|---|---|
| **Intrinsics RMS + parameter uncertainties** | RMS < 0.3 px good / < 1 px fail; `calibrateCameraExtended` std-devs | `zfcc-intrinsics` audit (separate tool) |
| **Intrinsics coverage gate** | X/Y fill ≥ 50%, ≥60% cells, size ratio ≥ 1.5, tilt ≥ 20° | `zfcc-intrinsics` (ROS X/Y/Size/Skew concept) |
| **Base-frame corner error** | mean/p95/max, mm | reported in the solve report |
| **Physical touch test** | < 3 mm good / > 5 mm re-capture | **manual** ruler check *after* the write |
| **Optional BA refinement** | nonlinear refine + covariance | `zfcc.refine` (`.[refine]`), off by default |

On noise-free synthetic eye-to-hand geometry (24 diverse poses), the solver recovers the ground-truth
extrinsic to **< 0.5 mm / < 0.05°**, with all five solvers agreeing to the same bound and the
independent robot-world cross-check recovering both transforms to **< 0.01 mm** — these exact bounds
are asserted in CI by `tests/test_handeye_synthetic.py`. Under realistic sub-mm detection noise
(0.3–0.5 mm / 0.03–0.05°, 20–24 poses) the end-to-end session solver still recovers the extrinsic to
**< 5 mm** and the pose-diversity gate refuses degenerate sets (asserted in
`tests/test_session.py` and `tests/test_handeye_synthetic.py::test_noise_degrades_gracefully`).

---

## Install

```bash
pip install -e .            # numpy, opencv-contrib-python, pyyaml
pip install -e ".[dev]"     # + pytest, ruff, pre-commit
```

Hardware capture additionally needs the **ZED SDK** (`pyzed`) and the Flexiv RDK (`flexivrdk`) or a
running [`flexiv-control`](https://pypi.org/project/flexiv-control/) `serve` daemon. Neither is
required to install the package, run the tests, or **solve from a saved session** — the hardware
shims use guarded imports.

## Quickstart

```bash
# 0. Verify the printed board matches your physical Calib.io target (counts, dict, square sizes!)
zfcc-render-board --board configs/board_calibio_9x14.yaml --out board.png

# 1. (optional) Audit the ZED factory intrinsics with ChArUco
zfcc-intrinsics  --zed configs/zed_2i_hd720.yaml --board configs/board_calibio_9x14.yaml --frames 20

# 2. Capture an eye-to-hand session: many DIVERSE flange poses, board always in view
zfcc-collect --session runs/s001 --board configs/board_calibio_9x14.yaml \
             --zed configs/zed_2i_hd720.yaml --robot configs/rizon4s.yaml --mode manual

# 3. Check coverage/diversity before you leave the robot
zfcc-inspect --session runs/s001

# 4. Solve offline + validate; writes T_base_zed2i.yaml only if the verdict isn't FAIL
zfcc-solve --session runs/s001 --board configs/board_calibio_9x14.yaml --out T_base_zed2i.yaml

# 5. Physically confirm the end-to-end accuracy (the number that predicts grasps)
zfcc-touch-test --calib T_base_zed2i.yaml --board configs/board_calibio_9x14.yaml \
                --zed configs/zed_2i_hd720.yaml --corner 0
```

Each console script is mirrored by a file in [`scripts/`](scripts/) (`python scripts/x.py …`).

## Output: drop-in `T_base_zed2i.yaml`

```yaml
T_base_zed2i:
  frame_convention: "T_A_B maps a point expressed in frame B into frame A: p_A = T_A_B @ p_B"
  parent_frame: flexiv_world
  child_frame: zed2i_camera_frame
  matrix: [[...4x4...]]            # base <- camera
  translation_xyz_m: [...]
  quaternion_wxyz: [...]           # scalar-first (Flexiv convention)
  inverse_T_camera_base: [[...]]
validation: { verdict: PASS, cross_solver_spread: {...}, axxb: {...}, leave_one_out: {...} }
```

Consume it as `p_base = T_base_zed2i.matrix @ [p_camera; 1]`.

## ⚠️ Before you trust any result

1. **Confirm board geometry.** A wrong `square_length_m` scales the whole extrinsic linearly. Measure
   your physical board and edit [`configs/board_calibio_9x14.yaml`](configs/board_calibio_9x14.yaml)
   (`squares_xy` is **(cols, rows)**; verify the ArUco dictionary too).
2. **Pair the FLANGE pose, not the TCP pose** (see method above).
3. **Make the poses diverse** — ≥20 poses, ≥3 non-parallel wrist-rotation axes, varied camera
   distance. The diversity gate will refuse a degenerate set; that is the point.
4. **Run the touch test.** A great reprojection RMS can still hide a bad chain.

## Documentation

- **[docs/TUTORIAL.md](docs/TUTORIAL.md)** — start here if this is your first ChArUco calibration:
  a detailed, end-to-end first-timer walkthrough (board prep → capture → solve → validate → touch
  test) with the verified numbers, the four day-one mistakes, and honest caveats on which thresholds
  are standards vs this repo's choices.
- **[docs/PRIOR_ART.md](docs/PRIOR_ART.md)** — how OpenCV, MoveIt Calibration, easy_handeye,
  industrial_calibration, Kalibr, multical and ROS `camera_calibration` do it, and why we chose what
  we chose.
- [METHOD](docs/METHOD.md) (the math) · [FRAMES](docs/FRAMES.md) (conventions) ·
  [PROCEDURE](docs/PROCEDURE.md) (terse checklist) · [PASS_BARS](docs/PASS_BARS.md) (gate definitions)
  · [VERSIONS](docs/VERSIONS.md) (OpenCV/ZED/RDK compatibility + distortion models).

## Repository layout

```
src/zfcc/        se3, board, detect, intrinsics, handeye, diversity, coverage, validate,
                 touch_test, refine, yaml_out, session, zed_io, robot_io, _cli
scripts/         thin CLI wrappers over zfcc._cli
configs/         board / zed / robot / pass_bars YAML
tests/           synthetic, hardware-free suite (round-trips a known extrinsic) — 81 tests
docs/            TUTORIAL, PRIOR_ART, METHOD, FRAMES, PROCEDURE, PASS_BARS, VERSIONS
```

## License

Apache-2.0 © 2026 Zihao Lu. See [LICENSE](LICENSE).
