"""
Interactive camera test utility.

Features:
- MJPEG preview stream (same style as voltage_mapping_interactive.py)
- Prints camera configuration and centroid threshold settings
- Interactive capture + centroid diagnostics
- Draws a circle on the preview stream at the detected centroid
"""

from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Optional

import cv2

from src import centroiding, picam
from src.constants import (
    LASER_CENTROID_INTENSITY_THRESHOLD,
    LASER_CENTROID_THRESHOLD_ENABLED,
)

_latest_jpeg: Optional[bytes] = None
_latest_lock = threading.Lock()
_stop_preview = threading.Event()
_overlay_centroid: Optional[tuple[float, float]] = None
_overlay_lock = threading.Lock()


class _PreviewHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *_args: Any) -> None:
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
            while not _stop_preview.is_set():
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


def _preview_capture_loop(cam: Any, quality: int, period_s: float) -> None:
    global _latest_jpeg
    while not _stop_preview.is_set():
        t0 = time.monotonic()
        try:
            gray = picam.get_gray_frame(cam)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            with _overlay_lock:
                cxy = _overlay_centroid
            if cxy is not None:
                cv2.circle(
                    bgr,
                    (int(round(cxy[0])), int(round(cxy[1]))),
                    10,
                    (0, 255, 0),
                    2,
                    lineType=cv2.LINE_AA,
                )
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if ok:
                with _latest_lock:
                    _latest_jpeg = buf.tobytes()
        except Exception as exc:
            print(f"[preview] capture error: {exc}")
        dt = time.monotonic() - t0
        to_sleep = period_s - dt
        if to_sleep > 0:
            time.sleep(to_sleep)


def _print_camera_details(cam: Any) -> None:
    print("--- Camera Settings ---")
    try:
        cfg = cam.camera_configuration()
        print(f"camera_configuration: {cfg}")
        main_cfg = cfg.get("main", {}) if isinstance(cfg, dict) else {}
        if isinstance(main_cfg, dict) and "size" in main_cfg:
            print(f"resolved resolution: {main_cfg['size']}")
    except Exception as exc:
        print(f"Could not read camera_configuration: {exc}")

    try:
        meta = cam.capture_metadata()
        print(f"capture_metadata: {meta}")
    except Exception as exc:
        print(f"Could not read capture_metadata: {exc}")


def _print_centroid_details() -> None:
    print("--- Centroiding Settings ---")
    print(f"threshold enabled: {LASER_CENTROID_THRESHOLD_ENABLED}")
    print(f"threshold value: {LASER_CENTROID_INTENSITY_THRESHOLD}")


def _analyze_capture(gray, roi_size: int) -> Optional[tuple[float, float]]:
    global _overlay_centroid
    h, w = gray.shape
    min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(gray)
    x0, y0 = max_loc

    half = roi_size // 2
    x_min = max(x0 - half, 0)
    x_max = min(x0 + half, w)
    y_min = max(y0 - half, 0)
    y_max = min(y0 + half, h)
    roi = gray[y_min:y_max, x_min:x_max]
    roi_min = float(roi.min()) if roi.size else float("nan")

    cxy = centroiding.find_laser_centroid(gray, roi_size=roi_size)

    print("--- Capture Diagnostics ---")
    print(f"highest pixel intensity (full frame): {float(max_val):.3f}")
    print(f"lowest pixel intensity (ROI): {roi_min:.3f}")
    print(f"lowest pixel intensity (full frame): {float(min_val):.3f}")
    if cxy is None:
        print("centroid: None (no valid weighted signal)")
    else:
        print(f"centroid: ({cxy[0]:.3f}, {cxy[1]:.3f}) px")

    with _overlay_lock:
        _overlay_centroid = cxy
    return cxy


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive camera test + centroid diagnostics.")
    parser.add_argument("--resolution", type=int, default=640, help="Camera width, height fixed at 480")
    parser.add_argument("--roi", type=int, default=50, help="Centroid ROI size (pixels)")
    parser.add_argument("--bind", type=str, default="0.0.0.0", help="Preview bind address")
    parser.add_argument("--port", type=int, default=8080, help="Preview HTTP port")
    parser.add_argument("--fps", type=float, default=24.0, help="Preview FPS")
    parser.add_argument("--quality", type=int, default=75, help="Preview JPEG quality (1-100)")
    args = parser.parse_args()

    cam = None
    server: Optional[HTTPServer] = None
    server_thread: Optional[threading.Thread] = None
    cap_thread: Optional[threading.Thread] = None
    try:
        _stop_preview.clear()
        cam = picam.init_camera(args.resolution)
        _print_camera_details(cam)
        _print_centroid_details()

        cap_thread = threading.Thread(
            target=_preview_capture_loop,
            args=(cam, int(args.quality), 1.0 / max(args.fps, 1.0)),
            daemon=True,
        )
        cap_thread.start()

        server = HTTPServer((args.bind, args.port), _PreviewHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()

        print("--- Preview Stream ---")
        print(f"Open stream: http://<this-pi>:{args.port}/")
        print("Commands: [Enter]=capture+centroid, q=quit")

        while True:
            cmd = input().strip().lower()
            if cmd in {"q", "quit", "exit"}:
                break
            gray = picam.get_gray_frame(cam)
            _analyze_capture(gray, int(args.roi))
            print("Commands: [Enter]=capture+centroid, q=quit")
        return 0
    finally:
        _stop_preview.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        if cap_thread is not None:
            cap_thread.join(timeout=2.0)
        picam.close_camera(cam)
        global _latest_jpeg
        with _latest_lock:
            _latest_jpeg = None


if __name__ == "__main__":
    raise SystemExit(main())
