# Method — eye-to-hand ChArUco calibration

## Setup

- **Camera**: fixed in the workspace (ZED 2i), observing the scene. *Eye-to-hand.*
- **Board**: a ChArUco target rigidly bolted to the robot **flange**.
- **Robot**: a Flexiv Rizon; we read its **flange** pose per capture.

Two rigid transforms are unknown and **constant** across all poses:

- `T_base_camera` — the camera in the robot base frame. **This is what we want.**
- `T_flange_board` — the board on the flange (a useful by-product / consistency handle).

## The constraint

For every captured pose `i`, the board's pose in the base frame can be written two ways:

```
via the camera:   T_base_board(i) = T_base_camera · T_cam_board(i)
via the robot:    T_base_board(i) = T_base_flange(i) · T_flange_board
```

Equating them gives the hand-eye constraint

```
T_base_camera · T_cam_board(i) = T_base_flange(i) · T_flange_board        (∀ i)
```

which is an `AX = XB`-type system once you difference consecutive poses. `T_base_camera` and
`T_flange_board` are solved jointly.

## Eye-to-hand via `cv2.calibrateHandEye`

OpenCV's `calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)` natively
solves the **eye-in-hand** case and returns `X = T_cam_gripper`. For a **fixed camera** (eye-to-hand)
we use the standard duality: feed the **inverted** robot poses

```
T_gripper2base := inv(T_base_flange)
T_target2cam   := T_cam_board          (as measured)
```

and the returned `X` is then **`T_base_camera`**. This inversion is the entire eye-to-hand trick and
is implemented in exactly one place — [`handeye.solve_eye_to_hand`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/handeye.py). The unit
test `test_inversion_sign_guard` deliberately feeds the wrong convention and asserts the result is
detectably wrong (> 5 mm vs the < 0.5 mm correct solve), because a flipped inversion still returns a
clean, plausible-looking matrix.

We run **all five** OpenCV solvers and report their spread:

| Solver | Note |
|---|---|
| `CALIB_HAND_EYE_TSAI` | classic, fast |
| `CALIB_HAND_EYE_PARK` | Lie-algebra |
| `CALIB_HAND_EYE_HORAUD` | quaternion |
| `CALIB_HAND_EYE_ANDREFF` | solves R,t simultaneously |
| `CALIB_HAND_EYE_DANIILIDIS` | dual-quaternion — **primary** (most rotation-robust) |

## Independent cross-check: `calibrateRobotWorldHandEye`

A second, structurally different algorithm solves the robot-world/hand-eye system `AX = ZB` in one
shot. Matching OpenCV's model equation `T_cam_world = T_cam_gripper · T_gripper_base · T_base_world`
to our constraint (with *world = board*) yields this bookkeeping:

```
input  world2cam    = T_cam_board          (as-is)
input  base2gripper = T_base_flange         (NOT inverted -> maps to T_gripper_base)
output gripper2cam  = T_camera_base   ->  T_base_camera  = inv(gripper2cam)
output base2world   = T_board_flange  ->  T_flange_board = inv(base2world)
```

On noise-free synthetic data this recovers both transforms to machine precision (asserted in
`test_robot_world_crosscheck_recovers_both`). If it disagrees with the primary hand-eye solution by
more than the bar, the geometry or pairing is suspect.

## Board pose per view

For each image: detect ChArUco corners (`CharucoDetector.detectBoard`), map them to object points
(`board.matchImagePoints`), and solve `T_cam_board` with `solvePnP(SOLVEPNP_IPPE)` followed by
`solvePnPRefineLM`. Distortion is passed as zeros because the ZED left stream is rectified. We record
the reprojection RMS per view and drop grazing/blurry outliers before the final solve.

## Why the previous (coplanar-marker) calibration was risky

Solving the camera pose from markers that all lie in the **table plane** is a *planar* PnP problem:
well-constrained in the image plane, poorly constrained in **depth and out-of-plane tilt**. The
extrinsic can therefore be accurate in X/Y yet off by **1–2 cm in Z** — and every reprojection check
still looks perfect, because reprojection only exercises the camera↔board leg, not the full
base↔camera chain. The fixes here are structural:

1. The board moves through a **3D volume** of flange poses (not one plane).
2. A **diversity gate** ([`diversity.assess_diversity`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/diversity.py)) measures rotation-
   axis spread and a coplanarity index and **refuses** a degenerate set.
3. A **physical touch test** measures the real base-frame error at the tool tip.
