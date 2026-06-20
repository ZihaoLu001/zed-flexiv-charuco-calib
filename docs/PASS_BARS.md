# Pass bars — what "PASS / WARN / FAIL" means

Every numeric threshold lives in [`configs/pass_bars.yaml`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/configs/pass_bars.yaml) and
[`config.PassBars`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/config.py); the diversity thresholds live in
[`config.DiversityGates`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/config.py). The grading logic is
[`session._grade`](https://github.com/ZihaoLu001/zed-flexiv-charuco-calib/blob/main/src/zfcc/session.py). Tighten for research-grade work; loosen only with a
documented reason.

## Verdicts

- **FAIL** — the calibration is not trustworthy; `zfcc-solve` refuses to write the YAML (use
  `--force` only to inspect a known-bad result).
- **WARN** — usable but not ideal; the YAML is written, with the reasons recorded under `validation`.
- **PASS** — all bars met.

## Bars

| Metric | Source | PASS | FAIL | Catches |
|---|---|---|---|---|
| poses | diversity | ≥ 20 | < 15 | too little data |
| distinct rotation axes | diversity | ≥ 3 | < 3 | single-axis (near-parallel) wrist motion |
| coplanarity index¹ | diversity | ≥ 0.04 | < 0.04 | near-planar pose set (the old-calibration failure) |
| max inter-pose rotation | diversity | ≥ 60° | < 30° | rotations too small to constrain the solve |
| five-solver translation spread | handeye | < 2 mm | > 3 mm | unstable geometry |
| five-solver rotation spread | handeye | < 0.2° | > 0.5° | unstable geometry |
| robot-world cross-check | handeye | agrees | — | independent-algorithm disagreement |
| AX=XB residual (max) | validate | — | > 2 mm | per-pose inconsistency / bad pairing |
| per-frame PnP RMS | detect | — | > 1 px | blurry / grazing views (dropped) |
| leave-one-out origin std | validate | — | > 5 mm | one pose leveraging the fit |
| **touch test** (tool tip) | physical | < 3 mm | > 5 mm | the whole base↔camera↔tool chain |

¹ Coplanarity index = smallest / largest singular value of the centred Nx3 matrix of flange
positions. ~0 ⇒ the positions lie on a plane (degenerate); larger ⇒ they fill a 3D volume.

## The one bar that ships with the robot

Reprojection-style metrics only exercise the camera↔board leg. The **touch test** is the only check
that exercises the entire chain a grasp uses (`base ← camera ← board`, then commanded to the tool).
A calibration can pass every numeric bar and still miss at the tip if, e.g., the board size was
mis-entered — which is exactly why the touch test is mandatory, not optional.
