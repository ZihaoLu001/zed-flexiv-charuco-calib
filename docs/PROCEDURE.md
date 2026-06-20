# Calibration procedure (operator guide)

A careful run takes ~20 minutes and yields a calibration you can grasp on. Rushing the pose-diversity
step is the single most common way to get a confident-but-wrong result.

## 0. Mount and verify the board

- Bolt the ChArUco board rigidly to the **flange** (no flex, no tape). It must not shift between
  poses.
- Render the board and compare it to your physical Calib.io target **square by square**:
  ```bash
  zfcc-render-board --board configs/board_calibio_9x14.yaml --out board.png
  ```
- Open [`configs/board_calibio_9x14.yaml`](../configs/board_calibio_9x14.yaml) and confirm:
  `squares_xy` is **(cols, rows)**, `square_length_m` and `marker_length_m` match the spec sheet, and
  `aruco_dict` is the dictionary actually printed on the board. A 1 mm error on a 40 mm square biases
  the whole result by 2.5 %.

## 1. (Optional) Audit intrinsics

```bash
zfcc-intrinsics --zed configs/zed_2i_hd720.yaml --board configs/board_calibio_9x14.yaml --frames 20
```

Move the board to fill all regions of the image (corners included), varying distance and tilt. Audit
mode confirms the ZED factory `K` reprojects sub-pixel and reports a free-solve delta. RMS should be
< 0.3 px; a free-solve `fx`/`cx` within ~1 px of factory is reassuring.

## 2. Capture diverse poses

```bash
zfcc-collect --session runs/s001 --board configs/board_calibio_9x14.yaml \
             --zed configs/zed_2i_hd720.yaml --robot configs/rizon4s.yaml --mode manual
```

Target **≥ 20 poses**. For each pose, keep the **entire board in view and in focus**, then vary:

- **Wrist orientation** across **≥ 3 non-parallel axes** (roll, pitch, yaw — not just one).
- **Distance** to the camera (near / mid / far) — this is what constrains depth.
- **Position** across the image (don't keep the board centered every time).
- Tilt the board **20–45°** in different directions (the coverage gate enforces a floor of **≥ 20°**).

Avoid: all poses at the same height (a plane), tiny rotations only, or the board filling < ¼ of the
frame.

## 3. Inspect before you leave the robot

```bash
zfcc-inspect --session runs/s001
```

Read the diversity verdict. **PASS** = go solve. **WARN** = usable but add a few more varied poses.
**FAIL** = the set is degenerate (too few poses, near-coplanar, or near-parallel rotation axes) — keep
capturing. Also scan the per-capture table: corners should be high (≳ 60), PnP RMS low (< 1 px),
focus (Laplacian variance) not collapsing.

## 4. Solve and validate (offline, repeatable)

```bash
zfcc-solve --session runs/s001 --board configs/board_calibio_9x14.yaml --out T_base_zed2i.yaml
```

This grades the pose set, drops per-frame outliers, runs all five hand-eye solvers + the robot-world
cross-check, computes AX=XB and leave-one-out stability, and **writes `T_base_zed2i.yaml` only if the
verdict isn't FAIL** (override with `--force`). Inspect `runs/s001/report.json` for every metric.
You can re-solve as many times as you like without the robot.

## 5. Touch test (do not skip)

```bash
zfcc-touch-test --calib T_base_zed2i.yaml --board configs/board_calibio_9x14.yaml \
                --zed configs/zed_2i_hd720.yaml --corner 0
```

The script prints a base-frame target derived from a board corner the camera sees. Jog the TCP tip to
that point and measure the real miss with a ruler / dial indicator. **< 3 mm** is good; **> 5 mm**
means re-capture (usually insufficient depth diversity) before trusting grasps. This is the metric
that would have caught the old ~1.5–2 cm height error.

## 6. Deploy

Copy `T_base_zed2i.yaml` to your consumer (e.g. ActAhead's `config/calibration/`). Apply it as
`p_base = T_base_zed2i.matrix @ [p_camera; 1]`.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `AttributeError: cv2.aruco.CharucoDetector` | `opencv-python` + `opencv-contrib-python` both installed | keep only `opencv-contrib-python` (`pip uninstall opencv-python`) |
| diversity FAIL: coplanarity | poses on one plane | vary camera distance & height |
| diversity FAIL: rotation axes | only one wrist axis used | add roll/pitch/yaw poses |
| diversity FAIL: distinct depths | every view at one working distance | capture near + mid + far |
| coverage WARN: width/height/cells | board never reached the frame edges | slide the board around the whole image |
| coverage WARN: size ratio / tilt | one distance / never tilted | add near+far and ±45° tilted views |
| five-solver spread large | bad pairings / too few poses | drop outliers, capture more |
| touch test bad, metrics good | weak depth constraint or board-size error | re-measure board; add near/far poses |
| no corners detected | wrong dictionary, `legacy_pattern`, or glossy mount | fix `aruco_dict`; `legacy_pattern: null` to auto-resolve; use a matte board |
