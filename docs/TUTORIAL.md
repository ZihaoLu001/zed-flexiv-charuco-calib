# Tutorial — your first ChArUco hand-eye calibration (ZED 2i + Flexiv Rizon)

This is a from-scratch walkthrough for someone using a ChArUco board for the **first time**. It takes
you from "I have a ZED 2i and a Rizon arm" to "I have a trustworthy `T_base_zed2i.yaml`", and explains
*why* at each step. Budget ~30–45 minutes for your first run; most of that is capturing good poses.

> **What we're building.** The ZED 2i is **fixed** in the workspace looking at the robot; the ChArUco
> board is bolted to the robot **flange**. This is *eye-to-hand* calibration. The output is the rigid
> transform `T_base_camera` (where the camera sits in the robot base frame), so a 3D point the camera
> sees can be commanded directly in robot coordinates. We solve it with the closed-form AX=XB hand-eye
> method (DANIILIDIS primary). Camera **intrinsics** on the ZED are *audited*, not overwritten — see §4.

The numbers in this guide are drawn from established practice (OpenCV, calib.io, MoveIt Calibration,
easy_handeye, ROS `camera_calibration`) and are cited in [PRIOR_ART.md](PRIOR_ART.md). Where a number
is *guidance* rather than a hard rule, or is a **choice this repo made**, it says so — don't treat
every threshold as a law of physics.

---

## ⚠️ Read this first — the four day-one mistakes

These are the failures first-timers hit in the first ten minutes. Each is cheap to avoid and expensive
to debug later.

1. **`opencv-python` and `opencv-contrib-python` installed together.** This is *the* most common
   crash: `cv2.aruco.CharucoDetector` throws `AttributeError`. Install **exactly one** —
   `opencv-contrib-python`. If you see that error, run `pip uninstall opencv-python` and keep the
   contrib build. (The ChArUco API lives in `objdetect`/contrib since OpenCV 4.7.)
2. **Wrong `squares_xy` axis order.** `squares_xy` is **(cols, rows) = (squaresX, squaresY)**. A board
   catalogued as "9×14" (9 rows × 14 cols) is `squares_xy: [14, 9]`. Get this wrong and detection
   silently maps corners to the wrong object points. **Always run `zfcc-render-board` and eyeball the
   result against your physical board before capturing anything** (step 1).
3. **A glossy / specular board mount.** ArUco markers need contrast; a shiny aluminium or laminated
   surface blows out under the ZED's exposure and you get **zero detections**. Use a **matte** print
   and a matte mount.
4. **Capturing while the arm is still moving.** Read the flange pose and the image at the **same
   instant**, with the arm **fully stopped**. Motion between the two corrupts every pose. The capture
   tool waits for you; let the arm settle (~1 s) before you record.

---

## 0. Install and sanity-check

```bash
pip install -e .          # numpy, opencv-contrib-python, pyyaml
pip install -e ".[dev]"   # + pytest, ruff (also pulls scipy, used by the optional refinement)
pytest -q                 # 81 synthetic tests, no hardware needed (just the pip deps above) — confirms your install works
```

Hardware capture also needs the **ZED SDK** (`pyzed`) and either the Flexiv RDK (`flexivrdk`) or a
running `flexiv-control serve` daemon. Neither is needed to install, test, or solve offline.

---

## 1. Choose and prepare the board

If you bought a Calib.io ChArUco target, you already have a good board — just read its spec sheet. If
you're choosing one:

- **Grid:** use an **asymmetric** grid (`squaresX ≠ squaresY`) so orientation is unambiguous. A
  `7×5` or `8×6` board is a fine first board. Interior corners = `(cols−1)·(rows−1)`.
- **Marker/square ratio:** the marker **must be strictly smaller** than the square. Anything in the
  ~**0.6–0.75** range works (OpenCV's own example uses 0.60; Calib.io ships ~0.73). The only hard
  requirement is that the value in your config matches the physical board — *don't reprint to hit a
  specific ratio.*
- **Dictionary:** pick the **smallest** ArUco dictionary that covers the board's markers (smaller
  dictionaries have larger inter-marker distance → fewer misreads). The dictionary is a **property of
  the printed board** — you must tell `zfcc` the exact one (Calib.io default is `DICT_5X5`).
- **Print & mount:** print at **exactly 100%** (disable "fit to page"), mount on something **flat and
  rigid** (≥5 mm aluminium-composite or ≥6 mm glass) with a **matte** finish. Non-flatness injects
  error proportional to the bow.
- **Measure it.** With calipers, measure the **actual** printed square and marker side in millimetres
  and put those true values in the config. *This sets the metric scale of every pose the calibration
  produces* — a 1 mm error on a 40 mm square biases everything by 2.5%.

Edit [`configs/board_calibio_9x14.yaml`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/configs/board_calibio_9x14.yaml):

```yaml
board:
  squares_xy: [14, 9]      # (cols, rows)  <- CONFIRM against your board
  square_length_m: 0.040   # measured with calipers
  marker_length_m: 0.030   # measured with calipers
  aruco_dict: DICT_5X5_100 # the dictionary your board was printed with
  legacy_pattern: null     # null = auto-resolve the OpenCV 4.6 parity flag on first capture
```

Now **verify the geometry**:

```bash
zfcc-render-board --board configs/board_calibio_9x14.yaml --out board.png
```

Open `board.png` and compare it square-by-square to your physical board: same number of squares, same
marker pattern, markers in the same cells. If it doesn't match, fix the config **now**.

> **Why the parity flag?** OpenCV 4.6 changed the ChArUco chessboard parity for **even** row counts.
> Boards printed before 4.6 (or by some third-party tools) need `legacy_pattern: true` or the
> corner→object-point mapping is silently wrong even though markers detect. Leaving it `null` lets
> `zfcc` pick the flag that detects more corners on your first frame. OpenCV 4.6 also flipped the board
> Z-axis to point *into* the plane — see the §6 axis sanity check.

---

## 2. Mount the board and fix the camera

- Bolt the board **rigidly to the flange**. It must not shift between poses (eye-to-hand assumes the
  board is fixed to the gripper).
- Mount the ZED on a **static** rig and don't touch it during capture. `zfcc` freezes the ZED's
  self-calibration so its factory `K` is constant across opens.
- Calibrate at roughly the **working distance** of your real task, and don't change focus/aperture
  afterward.
- Confirm `zfcc` reads the **flange** pose (it does by default), not a possibly-mis-set TCP — see
  [FRAMES.md](FRAMES.md) for why TCP would leak the tool offset into the result.

---

## 3. Capture diverse poses (the step that decides everything)

```bash
zfcc-collect --session runs/s001 --board configs/board_calibio_9x14.yaml \
             --zed configs/zed_2i_hd720.yaml --robot configs/rizon4s.yaml --mode manual
```

Hand-guide the arm to a new pose, let it **settle**, press ENTER to record, repeat. For each pose,
keep the **whole board in view and in focus**. Across the set, deliberately vary:

- **Wrist rotation** about **≥ 2–3 different axes** (roll *and* pitch *and* yaw), up to ~**90°** in
  both directions. Rotating about a single axis is *mathematically degenerate* — translation becomes
  unobservable. This is the most important kind of variety.
- **Working distance** (near / mid / far). Seeing the board at only one depth is a classic weak-Z set
  — exactly the failure mode behind the old table-marker calibration. `zfcc` **refuses** a set with
  fewer than 3 distinct depths.
- **Position in the frame** — push the board into every corner and edge, not just the center.
- **Tilt** — **20–45°** in both axes. Focal length and the principal point are only observable
  through foreshortening (tilted views), so flat-on views alone underconstrain the intrinsics. The
  coverage gate enforces a floor of **≥ 20°** on at least one view (`min_skew_deg`).

**How many poses?** This is *guidance, not consensus*: MoveIt's minimum is 5 and accuracy plateaus
around 12–15; calib.io's hard floor for intrinsics is 6; practical rigor is ~15–30. **This repo
chose** to FAIL below 15 and only call it ideal at 20+. Aim for **20 genuinely different** poses.

Avoid: all poses at the same height (a plane), tiny rotations only, the board filling < ¼ of the
frame, and — again — capturing mid-motion.

---

## 4. Run intrinsics — audit on the ZED, free for transparency

```bash
zfcc-intrinsics --zed configs/zed_2i_hd720.yaml --board configs/board_calibio_9x14.yaml --frames 20
```

**Why "audit" and not a fresh solve?** The ZED SDK's left image is already **rectified** against a
factory `K` that the **depth engine also uses**. If you computed your own `K` and overwrote it, RGB
and depth would desync. So the default **audit mode** seeds the factory `K`, fixes focal length +
principal point + zero distortion, and just **confirms** that the factory model reprojects the board
with small RMS. A free solve is reported alongside for transparency. The tool also prints a
**coverage report** (did the board reach the edges / vary depth / get tilted?) and saves a heatmap
`intrinsics_coverage.png` — green-ish everywhere is what you want.

> **If you ever calibrate a RAW (non-rectified) or wide lens** (not the ZED's rectified stream), use
> `--distortion-model`: `brown5` (default, normal lenses), `rational8` (wide / strong radial), or
> `fisheye` (best-effort; OpenCV's fisheye solver is fragile on planar ChArUco targets — see
> [VERSIONS.md](VERSIONS.md)). Don't enable `rational8` on a normal lens; it overfits.

**Reading the result:** RMS **< 0.3 px** is good, **< 1 px** acceptable, a **median > 1 px** signals
failure (these are the `intrinsics_rms_px_pass`/`_fail` bars in
[`configs/pass_bars.yaml`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/configs/pass_bars.yaml)). But — and this is the rule professionals repeat — **a low RMS does *not* prove a good
calibration**; it only shows the model can fit the data. That's why we also report **per-parameter
standard deviations** (is `fx`/`cx` actually well-constrained?) and the coverage metrics.

---

## 5. Solve the extrinsic hand-eye transform (offline, repeatable)

First, check what you captured *before* solving:

```bash
zfcc-inspect --session runs/s001     # per-pose corners/focus/PnP + the diversity verdict
```

A **PASS** diversity verdict means go; **WARN** means add a few more varied poses; **FAIL** means the
set is degenerate (too few poses, near-coplanar, single rotation axis, or only one working distance) —
keep capturing. Then solve:

```bash
zfcc-solve --session runs/s001 --board configs/board_calibio_9x14.yaml --out T_base_zed2i.yaml
```

This inverts each flange pose internally (the eye-to-hand trick), runs all five AX=XB solvers with
**DANIILIDIS** as primary, cross-checks with `calibrateRobotWorldHandEye`, drops per-frame outliers,
and **writes `T_base_zed2i.yaml` only if the verdict isn't FAIL** (override with `--force`). The full
report (every metric below) is saved to `runs/s001/report.json`.

> **Optional high-accuracy polish.** `from zfcc.refine import refine_extrinsic` runs a nonlinear
> least-squares refinement of `T_base_camera` + `T_flange_board` seeded by the DANIILIDIS solution,
> and reports parameter **covariance**. It needs `scipy` (`pip install '.[refine]'`). It minimizes the
> pose-consistency residual and gives you an uncertainty estimate; on a clean dataset the closed-form
> solution is already excellent, so treat this as a polish + uncertainty step, not a magic fix.

---

## 6. Validate before you trust it

Read the pass bars in `report.json` (all thresholds live in
[`configs/pass_bars.yaml`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/configs/pass_bars.yaml)):

| Metric | Bar | Effect on the write |
|---|---|---|
| cross-solver translation spread | < 2 mm pass / 3 mm fail | **FAIL** > 3 mm, WARN > 2 mm |
| cross-solver rotation spread | < 0.2° pass / 0.5° fail | **FAIL** > 0.5°, WARN > 0.2° |
| AX=XB residual (max) | < 2 mm | **FAIL** > 2 mm |
| leave-one-out origin std | < 5 mm | **FAIL** > 5 mm |
| robot-world cross-check | disagreement vs primary | WARN > 2 mm (cross-check, not a hard fail) |
| base-frame corner error | mm, mean/p95/max | advisory (reported, not gated) |
| intrinsics RMS | < 0.3 px good / < 1.0 px fail | the *separate* `zfcc-intrinsics` audit — not the solve verdict |

`zfcc-solve` refuses to write `T_base_zed2i.yaml` on any **FAIL** (override with `--force`); the
exact failing bars are listed in `report.json` under `verdict_reasons`.

> **Honest caveat:** the coplanarity-index bar (0.04) and the touch-test bars (< 3 mm good / > 5 mm
> fail) are **this repo's empirically-chosen acceptance bars**, not externally-standardized constants.
> They're sensible defaults tuned for a benchtop grasp task — adjust them in `pass_bars.yaml` for your
> tolerance and say so.

**Axis sanity check (do this once).** OpenCV 4.6 flipped the board Z-axis convention. On one capture,
draw the board frame and confirm Z points where you expect:

```python
import cv2, numpy as np
# det, board, K from your capture; T_cam_board from zfcc.detect.board_pose_pnp
cv2.drawFrameAxes(image_bgr, K, np.zeros(5), rvec, tvec, 0.05)
cv2.imwrite("axes.png", image_bgr)   # X red, Y green, Z blue — confirm Z sense
```

A wrong-handed board pose feeds a subtly wrong rotation into hand-eye; this check catches it.

---

## 7. Touch test, then ship

The numbers above only exercise the camera↔board leg. The **touch test** exercises the whole chain a
grasp actually uses (`base ← camera ← board`, then commanded to the tool):

```bash
zfcc-touch-test --calib T_base_zed2i.yaml --board configs/board_calibio_9x14.yaml \
                --zed configs/zed_2i_hd720.yaml --corner 0
```

It prints a base-frame target derived from a board corner the camera sees. Jog the TCP tip there and
measure the real miss with a ruler or dial indicator: **< 3 mm** is good, **> 5 mm** means re-capture
(usually insufficient depth diversity or a board-scale error). When it passes, copy
`T_base_zed2i.yaml` into your consumer (e.g. ActAhead's `config/calibration/`).

> **A note on residual error.** After the solver is perfect, the dominant real-world error is often
> the **robot side**: a Rizon 4s's *repeatability* is excellent (~0.05 mm, ISO 9283) but its *absolute* accuracy can
> be mm-level, and the flange-frame definition matters. If the touch test sits at a few mm despite
> clean solver metrics, that's likely robot absolute accuracy, not the calibration.

---

## Quick reference — verified numbers vs repo choices

| Quantity | Value | Status |
|---|---|---|
| min views (intrinsics) | 6 floor; 15–30 practical | guidance (calib.io / MoveIt) |
| board tilt | up to ±45° | guidance (calib.io) |
| rotation axes | ≥ 2 required; this repo gates ≥ 3 | requirement + repo choice |
| poses | 5 min / 12–15 plateau; this repo: 15 fail, 20 ideal | guidance + repo choice |
| marker/square ratio | ~0.6–0.75 (marker strictly < square) | guidance |
| intrinsics RMS | < 0.3 good / < 1.0 fail | **repo-chosen bar** (sub-px norm: calib.io) |
| coplanarity index / touch test | 0.04 / 3 mm / 5 mm | **repo-chosen bars** |
| distortion default | brown5 (k1,k2,p1,p2,k3) | standard |
| primary hand-eye solver | DANIILIDIS | matches MoveIt default |

See [PRIOR_ART.md](PRIOR_ART.md) for how this compares to other tools, [METHOD.md](METHOD.md) for the
math, [PROCEDURE.md](PROCEDURE.md) for the terse operator checklist, and [PASS_BARS.md](PASS_BARS.md)
for the gate definitions.
