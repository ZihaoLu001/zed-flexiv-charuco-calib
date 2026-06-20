# Frames & conventions

## Naming

`T_A_B` is a 4×4 homogeneous transform that maps a point expressed in frame **B** into frame **A**:

```
p_A = T_A_B · p_B
```

Composition reads right-to-left and cancels adjacent frames:
`T_A_C = T_A_B · T_B_C`. Inverse: `inv(T_A_B) = T_B_A`.

OpenCV's `X2Y` naming (e.g. `gripper2base`) means "transform that expresses X in Y" = maps points
from X into Y = `T_Y_X`. The mapping between the two conventions is handled inside
[`handeye.py`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/handeye.py); everywhere else in this repo uses `T_A_B`.

## Frames used

| Frame | Meaning |
|---|---|
| `base` / `flexiv_world` | robot base — the frame grasps are commanded in |
| `flange` | robot mechanical flange (tool mounting face); the **board** is here |
| `tcp` | tool center point = flange · tool offset; **not** used for calibration |
| `camera` / `zed2i_camera_frame` | ZED 2i left optical frame |
| `board` | ChArUco board origin (its first interior corner) |

Key transforms:

- `T_base_flange` — **read from the robot** each capture (Flexiv `flange_pose`).
- `T_cam_board` — **measured from the image** each capture (ChArUco + PnP).
- `T_base_camera` — **solved** (the deliverable).
- `T_flange_board` — **solved** (by-product / consistency check).

## Quaternions

Scalar-first **`(w, x, y, z)`**, the Flexiv / RDK convention. A "pose7" vector is
`[x, y, z, qw, qx, qy, qz]` — exactly the layout of Flexiv `tcp_pose` / `flange_pose`. All
conversions live in [`se3.py`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/se3.py) (`pose7_to_T`, `T_to_pose7`, `quat_wxyz_to_R`,
`R_to_quat_wxyz`). Rigid inverses use `(Rᵀ, −Rᵀt)`, never a dense `np.linalg.inv`.

## Units

Metres and radians throughout. The board's `square_length_m` / `marker_length_m` set the metric
scale of the entire calibration — **measure your physical board** and set them correctly, or the
extrinsic is scaled by the same factor.

## Flange vs TCP — why it matters

The board is bolted to the **flange**, so hand-eye must be paired with `T_base_flange`. If you pair
with `T_base_tcp` instead, the constant tool offset `T_flange_tcp` is folded into the solved
`T_flange_board` *and* biases `T_base_camera` — silently, by exactly the tool offset. The Flexiv RDK
exposes the flange pose as `RobotStates.flange_pose`; if a particular build lacks it, set the TCP to
the flange (zero tool) so `tcp_pose == flange_pose`. See [`robot_io.py`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/robot_io.py).
