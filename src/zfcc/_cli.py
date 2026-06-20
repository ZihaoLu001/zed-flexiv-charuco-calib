"""Console entry points (installed as zfcc-render-board / zfcc-intrinsics / zfcc-collect /
zfcc-solve / zfcc-touch-test / zfcc-inspect). The hardware-touching commands import pyzed / flexivrdk
lazily inside the function so the package installs and the offline commands run without them.

The thin ``scripts/*.py`` files call straight into these so both ``python scripts/x.py`` and the
installed console scripts share one implementation.
"""
from __future__ import annotations

import argparse
import json

import numpy as np


def render_board_main():
    from .board import make_board, n_interior_corners, render_board_png
    from .config import BoardConfig

    ap = argparse.ArgumentParser(description="Render the configured ChArUco board to a PNG.")
    ap.add_argument("--board", default="configs/board_calibio_9x14.yaml")
    ap.add_argument("--out", default="board.png")
    ap.add_argument("--px-per-square", type=int, default=120)
    a = ap.parse_args()
    cfg = BoardConfig.load(a.board)
    board = make_board(cfg)
    path = render_board_png(board, a.out, px_per_square=a.px_per_square)
    cols, rows = board.getChessboardSize()
    print(f"board: {cols}x{rows} squares, {n_interior_corners(board)} interior corners")
    print(f"square={cfg.square_length_m} m  marker={cfg.marker_length_m} m  dict={cfg.aruco_dict}")
    print(f"wrote {path} -- hold it next to the physical board and confirm every square matches")


def inspect_main():
    from .config import DiversityGates
    from .diversity import assess_diversity
    from .session import CalibrationSession

    ap = argparse.ArgumentParser(description="Inspect a captured session without solving.")
    ap.add_argument("--session", required=True)
    a = ap.parse_args()
    sess = CalibrationSession.load(a.session)
    print(f"session: {a.session}")
    print(f"  zed_serial={sess.zed_serial}  image_size={sess.image_size}")
    print(f"  captures={len(sess.captures)}  usable={len(sess.usable())}")
    print(f"  {'idx':>3} {'corners':>7} {'pnp_px':>7} {'focus':>8}")
    for c in sess.captures:
        rms = "-" if c.pnp_rms_px is None else f"{c.pnp_rms_px:.3f}"
        foc = "-" if c.laplacian_var is None else f"{c.laplacian_var:.0f}"
        print(f"  {c.index:>3} {c.n_corners:>7} {rms:>7} {foc:>8}")
    usable = sess.usable()
    if len(usable) >= 3:
        div = assess_diversity([c.T_base_flange for c in usable], DiversityGates(),
                               board_poses_T=[c.T_cam_board_mat for c in usable])
        print(f"\ndiversity verdict: {div.verdict}")
        for r in div.reasons:
            print(f"  - {r}")
        print(f"  rotation_axes={div.rotation_axes}  coplanarity={div.coplanarity_index:.4f}  "
              f"distinct_depths={div.distinct_depths}  "
              f"max_interpose_rot={div.max_interpose_rotation_deg:.1f}deg  "
              f"span={div.translation_span_m:.3f}m")
    else:
        print("\nnot enough usable poses yet (need >=3).")


def solve_main():
    from .board import make_board
    from .config import BoardConfig, DiversityGates, PassBars
    from .session import CalibrationSession, solve_session
    from .yaml_out import write_calibration_yaml

    ap = argparse.ArgumentParser(description="Solve eye-to-hand offline + validate + write YAML.")
    ap.add_argument("--session", required=True)
    ap.add_argument("--board", default="configs/board_calibio_9x14.yaml")
    ap.add_argument("--pass-bars", default="configs/pass_bars.yaml")
    ap.add_argument("--out", default="T_base_zed2i.yaml")
    ap.add_argument("--report", default=None)
    ap.add_argument("--no-drop-outliers", action="store_true")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    sess = CalibrationSession.load(a.session)
    board = make_board(BoardConfig.load(a.board))
    obj = board.getChessboardCorners()
    bars = PassBars.load(a.pass_bars)
    report = solve_session(sess, obj, gates=DiversityGates(), bars=bars,
                           drop_outliers=not a.no_drop_outliers)
    report_path = a.report or f"{a.session.rstrip('/')}/report.json"
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(f"verdict: {report['verdict']}   (usable poses: {report.get('n_usable')})")
    for k in ("diversity", "cross_solver_spread", "robot_world_crosscheck", "axxb",
              "base_frame_corner_error", "leave_one_out"):
        if k in report:
            print(f"  {k}: {report[k]}")
    print(f"report -> {report_path}")
    if report["verdict"] == "FAIL" and not a.force:
        print("REFUSING to write T_base_zed2i.yaml (verdict FAIL). Re-capture or pass --force.")
        raise SystemExit(2)
    if "T_base_camera" in report:
        out = write_calibration_yaml(
            a.out, report["T_base_camera"], serial=sess.zed_serial,
            validation={k: report[k] for k in
                        ("verdict", "diversity", "cross_solver_spread", "robot_world_crosscheck",
                         "axxb", "base_frame_corner_error", "leave_one_out", "n_usable", "dropped")
                        if k in report})
        print(f"calibration -> {out}")


def collect_main():
    from .board import make_board, make_detector, resolve_legacy_pattern
    from .config import BoardConfig, RobotConfig, ZedConfig
    from .detect import board_pose_pnp, detect_charuco
    from .robot_io import FlangePoseReader
    from .session import CalibrationSession, Capture
    from .zed_io import ZedCamera

    ap = argparse.ArgumentParser(description="Capture an eye-to-hand session (ZED + Flexiv flange).")
    ap.add_argument("--session", required=True)
    ap.add_argument("--board", default="configs/board_calibio_9x14.yaml")
    ap.add_argument("--zed", default="configs/zed_2i_hd720.yaml")
    ap.add_argument("--robot", default="configs/rizon4s.yaml")
    ap.add_argument("--mode", choices=["manual", "auto"], default="manual")
    ap.add_argument("--transport", choices=["rdk", "remote"], default="rdk")
    ap.add_argument("--no-images", action="store_true")
    a = ap.parse_args()

    bcfg = BoardConfig.load(a.board)
    board = make_board(bcfg)
    detector = make_detector(board)
    reader = FlangePoseReader(RobotConfig.load(a.robot), transport=a.transport).open()
    sess = CalibrationSession(root=a.session, board=bcfg.__dict__.copy())
    sess.dir.mkdir(parents=True, exist_ok=True)

    def record(idx, frame):
        det = detect_charuco(frame.gray, board, detector)
        T_cb, rms = None, None
        if det.ok:
            try:
                T, rms = board_pose_pnp(det, board, frame.K, D=None)
                T_cb = T.tolist()
            except Exception as e:
                print(f"  pose#{idx}: PnP failed ({e})")
        pose7 = reader.read_flange_pose7().tolist()
        img_path = None
        if not a.no_images and frame.bgr is not None:
            import cv2
            img_path = str(sess.dir / f"capture_{idx:03d}.png")
            cv2.imwrite(img_path, frame.bgr)
        sess.add(Capture(index=idx, image_path=img_path, flange_pose7=pose7,
                         n_corners=det.n_corners,
                         pnp_rms_px=(None if rms is None else float(rms)),
                         T_cam_board=T_cb, laplacian_var=det.laplacian_var))
        print(f"  pose#{idx}: corners={det.n_corners} "
              f"pnp_rms={'-' if rms is None else f'{rms:.3f}px'} "
              f"flange_xyz={np.round(pose7[:3], 4).tolist()}")

    with ZedCamera(ZedConfig.load(a.zed)) as cam:
        sess.zed_serial = cam.serial
        sess.factory_K = cam.K.tolist()
        sess.image_size = list(cam.image_size)
        if bcfg.legacy_pattern is None:
            print(f"resolved legacy_pattern={resolve_legacy_pattern(board, cam.grab().gray)}")
        idx = 0
        if a.mode == "manual":
            print("MANUAL capture: hand-guide to a new diverse pose, ENTER to record, q to finish.")
            while True:
                if input(f"[{idx}] ENTER=record / q=quit > ").strip().lower() == "q":
                    break
                record(idx, cam.grab())
                idx += 1
        else:
            targets = RobotConfig.load(a.robot).joint_targets
            if not targets:
                raise SystemExit("auto mode needs robot.joint_targets")
            for idx, _ in enumerate(targets):
                input(f"move to target {idx} then ENTER > ")
                record(idx, cam.grab())

    path = sess.save()
    print(f"\nsaved {path}: {len(sess.captures)} captures, {len(sess.usable())} pose-solved.")
    print(f"next: zfcc-inspect --session {a.session}")


def intrinsics_main():
    from .board import make_board, make_detector, resolve_legacy_pattern
    from .config import BoardConfig, CoverageGates, ZedConfig
    from .coverage import coverage_report, render_coverage_heatmap
    from .detect import board_pose_pnp, detect_charuco
    from .intrinsics import audit_against_factory, calibrate_intrinsics
    from .yaml_out import write_intrinsics_yaml
    from .zed_io import ZedCamera

    ap = argparse.ArgumentParser(description="Audit (or freely solve) ZED 2i intrinsics via ChArUco.")
    ap.add_argument("--zed", default="configs/zed_2i_hd720.yaml")
    ap.add_argument("--board", default="configs/board_calibio_9x14.yaml")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--mode", choices=["audit", "free"], default="audit")
    ap.add_argument("--distortion-model", choices=["brown5", "rational8", "fisheye"],
                    default="brown5", help="free-mode distortion model (ignored in audit mode)")
    ap.add_argument("--out", default="zed2i_intrinsics.yaml")
    a = ap.parse_args()

    board = make_board(BoardConfig.load(a.board))
    detector = make_detector(board)
    dets = []
    with ZedCamera(ZedConfig.load(a.zed)) as cam:
        factory_K, image_size, serial = cam.K.copy(), cam.image_size, cam.serial
        if BoardConfig.load(a.board).legacy_pattern is None:
            resolve_legacy_pattern(board, cam.grab().gray)
        print(f"capturing {a.frames} frames -- span the whole image with the board")
        for i in range(a.frames):
            input(f"[{i}] place board, ENTER to grab > ")
            det = detect_charuco(cam.grab().gray, board, detector)
            print(f"   corners={det.n_corners} focus={det.laplacian_var:.0f}")
            if det.ok:
                dets.append(det)

    # coverage assessment (X/Y/Size/Skew) before trusting the solve; skew needs board poses
    board_poses = []
    for det in dets:
        try:
            T, _ = board_pose_pnp(det, board, factory_K, D=None)
            board_poses.append(T)
        except Exception:
            pass
    cov = coverage_report(dets, image_size, board_poses_T=board_poses or None, gates=CoverageGates())
    print(f"\ncoverage: {cov.as_dict()}")
    for r in cov.reasons:
        print(f"  - {r}")
    try:
        print(f"coverage heatmap -> {render_coverage_heatmap(cov, 'intrinsics_coverage.png')}")
    except Exception as e:
        print(f"  (heatmap skipped: {e})")

    res = calibrate_intrinsics(dets, board, image_size, K0=factory_K, mode=a.mode,
                               distortion_model=a.distortion_model)
    from .config import PassBars
    bars = PassBars()
    rms_verdict = ("PASS" if res.rms_px <= bars.intrinsics_rms_px_pass
                   else "FAIL" if res.rms_px > bars.intrinsics_rms_px_fail else "WARN")
    print(f"\nmode={res.mode}  model={res.distortion_model}  views={res.n_views}  "
          f"reprojection_rms={res.rms_px:.4f}px  [{rms_verdict}: pass<={bars.intrinsics_rms_px_pass} "
          f"fail>{bars.intrinsics_rms_px_fail}]")
    print(f"K=\n{np.round(res.K, 3)}")
    if res.param_std:
        print(f"param std-dev (uncertainty): {res.as_dict()['param_std']}")
    audit = None
    if a.mode == "audit":
        free = calibrate_intrinsics(dets, board, image_size, K0=factory_K, mode="free")
        audit = audit_against_factory(free, factory_K)
        print(f"free-solve delta vs factory: {audit}")
    print(f"intrinsics -> {write_intrinsics_yaml(a.out, res, serial=serial, factory_audit=audit)}")


def touch_test_main():
    import yaml

    from .board import make_board, make_detector
    from .config import BoardConfig, ZedConfig
    from .detect import board_pose_pnp, detect_charuco
    from .touch_test import board_corner_in_camera, camera_point_to_base, touch_error
    from .zed_io import ZedCamera

    ap = argparse.ArgumentParser(description="Physical touch test of a solved T_base_zed2i.yaml.")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--board", default="configs/board_calibio_9x14.yaml")
    ap.add_argument("--zed", default="configs/zed_2i_hd720.yaml")
    ap.add_argument("--corner", type=int, default=0)
    a = ap.parse_args()

    T_bc = np.asarray(yaml.safe_load(open(a.calib, encoding="utf-8"))["T_base_zed2i"]["matrix"],
                      dtype=float)
    board = make_board(BoardConfig.load(a.board))
    detector = make_detector(board)
    objp = board.getChessboardCorners()
    with ZedCamera(ZedConfig.load(a.zed)) as cam:
        frame = cam.grab()
        det = detect_charuco(frame.gray, board, detector)
        if not det.ok:
            raise SystemExit("board not detected; reposition it")
        T_cam_board, rms = board_pose_pnp(det, board, frame.K, D=None)
        print(f"board detected: corners={det.n_corners} pnp_rms={rms:.3f}px")
        p_cam = board_corner_in_camera(T_cam_board, objp, a.corner)
        p_base = camera_point_to_base(T_bc, p_cam)
    print(f"\nTOUCH TARGET (board corner #{a.corner}):")
    print(f"  camera-frame XYZ (m): {np.round(p_cam, 4).tolist()}")
    print(f"  -> base-frame  XYZ (m): {np.round(p_base, 4).tolist()}")
    s = input("measured base XYZ as 'x y z' metres (ENTER to skip) > ").strip()
    if s:
        r = touch_error(p_base, np.array([float(v) for v in s.split()], dtype=float))
        print(f"  touch error: {r.error_mm:.1f} mm")
