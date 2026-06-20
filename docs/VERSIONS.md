# Versions & compatibility

## OpenCV — the 4.6 → 4.7 API break (this matters)

The ArUco/ChArUco API was rewritten in **OpenCV 4.7** (ChArUco also moved from `contrib` into the
main `objdetect` module in 4.7). This repo targets the **new** `objdetect` API only and requires
`opencv-contrib-python >= 4.7` (verified on **4.12.0**).

| Removed (R) / moved+deprecated (D) in 4.7 | Replacement (used here) |
|---|---|
| `cv2.aruco.CharucoBoard_create(sx, sy, sl, ml, dict)` (R) | `cv2.aruco.CharucoBoard((cols, rows), sl, ml, dict)` |
| `cv2.aruco.Dictionary_get(...)` (R) | `cv2.aruco.getPredefinedDictionary(...)` |
| `cv2.aruco.detectMarkers` + `interpolateCornersCharuco` (R) | `cv2.aruco.CharucoDetector(board).detectBoard(img)` |
| `cv2.aruco.calibrateCameraCharuco(...)` (D) | `board.matchImagePoints(...)` → `cv2.calibrateCameraExtended(...)` |
| `cv2.aruco.estimatePoseCharucoBoard(...)` (D) | `board.matchImagePoints(...)` → `cv2.solvePnP(...)` |

`calibrateCameraCharuco` was **not cleanly deleted** — it was relocated into `objdetect` and marked
**deprecated** (use `matchImagePoints` + `solvePnP`/`calibrateCamera`). The `AttributeError` people
hit calling it on 4.7+ is almost always an **install conflict**: `opencv-python` and
`opencv-contrib-python` co-installed. **Install exactly one** (`opencv-contrib-python`); if
`cv2.aruco.CharucoDetector` raises `AttributeError`, you have both — `pip uninstall` the plain one.

Note the constructor takes `(cols, rows)` = `(squaresX, squaresY)`. A board catalogued as "9×14"
(9 rows × 14 cols) is configured as `squares_xy: [14, 9]`.

### Distortion models & wide-FOV (free intrinsics mode)

`brown5` (k1,k2,p1,p2,k3) is the default and correct for normal lenses; `rational8`
(`CALIB_RATIONAL_MODEL`: k1..k6 + p1,p2) is for wide / strong-radial lenses (it overfits a normal
lens). Both use `cv2.calibrateCameraExtended`, which also returns **per-parameter standard
deviations** (uncertainties) — we surface these, because a low RMS alone does not prove a good
calibration. `fisheye` (the separate equidistant model, k1..k4) is **best-effort only**:
`cv2.fisheye.calibrate`'s extrinsic initialization is documented to fail data-dependently on
**planar** targets like a ChArUco board (it frequently fails and succeeds only intermittently on
planar ChArUco data), so this branch raises a clear, actionable error rather than a cryptic OpenCV
assertion.
For true fisheye / >~120–150° FOV lenses, prefer a dedicated tool (Kalibr) or a non-planar target.

`setLegacyPattern(True/False)` toggles the pre-4.6 chessboard parity; some third-party boards need it.
Leave `legacy_pattern: null` in the config to auto-resolve it on the first capture (the flag that
detects more corners wins).

## ZED 2i / ZED SDK

- The **left** stream is rectified ⇒ distortion ≈ 0; the factory `K` is read from the SDK and is the
  model the depth engine uses. Intrinsics default to **audit** (confirm, don't overwrite).
- `depth_mode: NEURAL` (the `ULTRA` mode is deprecated in SDK 5.x).
- `camera_disable_self_calib: true` freezes `K` across opens for reproducibility.
- Capture uses the Python `pyzed` API, installed via the ZED SDK (not from PyPI).

## Flexiv RDK / flexiv-control

- Poses are `[x, y, z, qw, qx, qy, qz]`, metres + scalar-first unit quaternion.
- Calibration reads the **flange** pose (`RobotStates.flange_pose`), not `tcp_pose`.
- Two transports: direct `flexivrdk.Robot`, or the
  [`flexiv-control`](https://pypi.org/project/flexiv-control/) `serve` daemon over its socket.

## Python

Tested on CPython **3.9 / 3.11 / 3.12** in CI. The math core is pure NumPy; OpenCV is imported lazily
so the package imports (and the synthetic tests that don't need cv2) work without it.
