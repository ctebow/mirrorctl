#!/usr/bin/env python3
"""
Interactive end-to-end camera calibration for Raspberry Pi.

Flow:
1) Start a browser-viewable MJPEG stream.
2) Prompt for camera/capture settings (with defaults).
3) Capture calibration images interactively (s/save, q/quit) while stream stays live.
4) Run lens calibration + homography and save config/camera_params.npz.

This script always enables legacy Charuco board behavior for compatibility with
older board layouts.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src import picam

# Package import when loaded as config.calibrate_camera_interactive; same-dir when run as a script.
if __package__ in (None, ""):
    _CFG = Path(__file__).resolve().parent
    if str(_CFG) not in sys.path:
        sys.path.insert(0, str(_CFG))
    import calibrate_picam  # sibling module when run as script (not as package)
else:
    from . import calibrate_picam

_CONFIG_DIR = Path(__file__).resolve().parent
_SAVE_DIR = _CONFIG_DIR / "calib_images"
_FILE_PREFIX = "charuco_shot"

_latest_jpeg: Optional[bytes] = None
_latest_gray: Optional[np.ndarray] = None
_latest_bgr: Optional[np.ndarray] = None
_latest_lock = threading.Lock()
_stop_capture = threading.Event()


def _prompt_with_default(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return raw if raw else default


def _prompt_int(prompt: str, default: int, *, minimum: int | None = None) -> int:
    while True:
        raw = _prompt_with_default(prompt, str(default))
        try:
            val = int(raw)
            if minimum is not None and val < minimum:
                raise ValueError
            return val
        except ValueError:
            min_msg = f" (>= {minimum})" if minimum is not None else ""
            print(f"Please enter a valid integer{min_msg}.", file=sys.stderr)


def _prompt_float(prompt: str, default: float, *, minimum: float | None = None) -> float:
    while True:
        raw = _prompt_with_default(prompt, str(default))
        try:
            val = float(raw)
            if minimum is not None and val < minimum:
                raise ValueError
            return val
        except ValueError:
            min_msg = f" (>= {minimum})" if minimum is not None else ""
            print(f"Please enter a valid number{min_msg}.", file=sys.stderr)


def _prompt_bool(prompt: str, default: bool) -> bool:
    default_str = "y" if default else "n"
    while True:
        raw = _prompt_with_default(f"{prompt} (y/n)", default_str).lower()
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("Please answer y or n.", file=sys.stderr)


def _capture_loop(picam2, jpeg_quality: int, target_period_s: float) -> None:
    global _latest_jpeg, _latest_gray, _latest_bgr
    while not _stop_capture.is_set():
        t0 = time.monotonic()
        try:
            gray = picam.get_gray_frame(picam2)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            if ok:
                with _latest_lock:
                    _latest_gray = gray
                    _latest_bgr = bgr
                    _latest_jpeg = buf.tobytes()
        except Exception as exc:
            print(f"[interactive_calibration] capture error: {exc}", file=sys.stderr)

        elapsed = time.monotonic() - t0
        sleep_for = target_period_s - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


class _MJPEGHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, _format, *_args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path not in ("/", "/stream"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while not _stop_capture.is_set():
                with _latest_lock:
                    chunk = _latest_jpeg
                if chunk:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(chunk)
                    self.wfile.write(b"\r\n")
                time.sleep(0.001)
        except (BrokenPipeError, ConnectionResetError):
            pass


def _next_index(prefix: str, save_dir: Path) -> int:
    existing = sorted(save_dir.glob(f"{prefix}_*.jpg"))
    max_idx = -1
    for p in existing:
        stem = p.stem
        suffix = stem.replace(f"{prefix}_", "", 1)
        if suffix.isdigit():
            max_idx = max(max_idx, int(suffix))
    return max_idx + 1


def _save_current_frame(path: Path, grayscale_mode: bool) -> bool:
    with _latest_lock:
        gray = None if _latest_gray is None else _latest_gray.copy()
        bgr = None if _latest_bgr is None else _latest_bgr.copy()

    if gray is None or bgr is None:
        print("No frame available yet; wait a moment and try again.", file=sys.stderr)
        return False

    out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if grayscale_mode else bgr
    ok = cv2.imwrite(str(path), out)
    if not ok:
        print(f"Failed to write image: {path}", file=sys.stderr)
        return False
    return True


def _run_calibration_pipeline(
    image_paths: list[Path],
    *,
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dict_type: int,
    homography_ref: Path | None,
) -> None:
    board, dictionary, detector = calibrate_picam.make_charuco_board(
        squares_x,
        squares_y,
        square_length,
        marker_length,
        dict_type,
        legacy_pattern=True,
    )

    rms, mtx, dist = calibrate_picam.calibrate_lens(image_paths, board, detector, dictionary)

    ref_path = homography_ref if homography_ref is not None else image_paths[0]
    ref = cv2.imread(str(ref_path))
    if ref is None:
        raise RuntimeError(f"Could not read homography reference image: {ref_path}")
    H, _ = calibrate_picam.get_homography_matrix(ref, mtx, dist, board, detector)
    if H is None:
        raise RuntimeError(
            "ChArUco board not detected in homography reference image; cannot compute H."
        )

    np.savez(calibrate_picam.OUTPUT_NPZ, mtx=mtx, dist=dist, rms=rms, H=H)
    print(f"Calibration saved to {calibrate_picam.OUTPUT_NPZ}")
    print(f"RMS reprojection error: {rms:.4f} px")
    print(f"Homography reference: {ref_path}")


def main() -> int:
    global _latest_jpeg, _latest_gray, _latest_bgr

    parser = argparse.ArgumentParser(
        description="Interactive Pi preview + capture + lens/homography calibration.",
    )
    parser.add_argument("--bind", default="0.0.0.0", help="Preview server bind address.")
    parser.add_argument("--port", type=int, default=8080, help="Preview server port.")
    parser.add_argument(
        "--width",
        type=int,
        default=picam.DEFAULT_FRAME_SIZE[0],
        help=f"Frame width (height 480), default {picam.DEFAULT_FRAME_SIZE[0]}.",
    )
    parser.add_argument("--fps", type=float, default=24.0, help="Preview capture rate.")
    parser.add_argument("--quality", type=int, default=75, help="Preview JPEG quality (1-95).")
    parser.add_argument(
        "--grayscale-captures",
        action="store_true",
        help="Save grayscale-derived BGR JPEGs (same as get_calib_photos.py).",
    )
    parser.add_argument("--squares-x", type=int, default=calibrate_picam.SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=calibrate_picam.SQUARES_Y)
    parser.add_argument("--square-length", type=float, default=calibrate_picam.SQUARE_LENGTH)
    parser.add_argument("--marker-length", type=float, default=calibrate_picam.MARKER_LENGTH)
    parser.add_argument("--dict", type=int, default=int(calibrate_picam.DICT_TYPE))
    parser.add_argument(
        "--homography-ref",
        type=Path,
        default=None,
        help="Optional explicit reference image for homography; default first captured image.",
    )
    args = parser.parse_args()

    print("--- Interactive Camera Calibration ---")
    print("Legacy Charuco mode: FORCED ON for this script.")
    print("Open on your Mac browser after startup:")
    print(f"  http://<this-pi>:{args.port}/")
    print()

    frame_width = _prompt_int("Preview/capture width", args.width, minimum=1)
    fps = _prompt_float("Preview fps", args.fps, minimum=1.0)
    quality = _prompt_int("Preview JPEG quality", args.quality, minimum=1)
    grayscale_mode = _prompt_bool("Save captures from grayscale stream", args.grayscale_captures)

    frame_size = picam.normalize_resolution(frame_width)
    target_period = 1.0 / max(fps, 1.0)

    picam2 = None
    cap_thread = None
    server = None
    server_thread = None
    captured_paths: list[Path] = []

    _SAVE_DIR.mkdir(parents=True, exist_ok=True)
    next_idx = _next_index(_FILE_PREFIX, _SAVE_DIR)

    try:
        picam2 = picam.init_camera(frame_size)
        cap_thread = threading.Thread(
            target=_capture_loop,
            args=(picam2, int(quality), target_period),
            daemon=True,
        )
        cap_thread.start()

        server = HTTPServer((args.bind, args.port), _MJPEGHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()

        print(f"Preview server running at http://<this-pi>:{args.port}/")
        print(f"Resolution: {frame_size}, ~{fps} fps, quality {quality}")
        print(f"Saving under: {_SAVE_DIR}")
        print("Commands:  s / save  -> save frame    q / quit  -> finish capture and calibrate")
        print()

        while True:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd in ("s", "save"):
                file_path = _SAVE_DIR / f"{_FILE_PREFIX}_{next_idx:02d}.jpg"
                if _save_current_frame(file_path, grayscale_mode):
                    captured_paths.append(file_path)
                    next_idx += 1
                    print(f"Saved: {file_path}")
                continue
            if cmd == "":
                continue
            print(f"Unknown command: {line!r} -- use s or q", file=sys.stderr)

        if len(captured_paths) < calibrate_picam.MIN_CALIB_IMAGES:
            print(
                f"Need at least {calibrate_picam.MIN_CALIB_IMAGES} captured images; got {len(captured_paths)}.",
                file=sys.stderr,
            )
            return 2

        print()
        print("Running lens calibration + homography...")
        _run_calibration_pipeline(
            captured_paths,
            squares_x=args.squares_x,
            squares_y=args.squares_y,
            square_length=args.square_length,
            marker_length=args.marker_length,
            dict_type=args.dict,
            homography_ref=args.homography_ref,
        )
        print("Done: camera_params.npz is ready for mapping.")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        _stop_capture.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        if cap_thread is not None:
            cap_thread.join(timeout=2.0)
        picam.close_camera(picam2)
        with _latest_lock:
            _latest_jpeg = None
            _latest_gray = None
            _latest_bgr = None


if __name__ == "__main__":
    raise SystemExit(main())
