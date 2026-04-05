"""
Live MJPEG preview server for headless Pi → view in a browser on your Mac.

No recording; use this only to frame/focus, then use get_calib_photos.py for saves.

On Pi:
  python3 config/preview_stream.py --bind 0.0.0.0 --port 8080

On Mac browser:
  http://<pi-ip>:8080/

Stop with Ctrl+C on the Pi. Same Picamera2 config as mapping (RGB888 → gray → BGR JPEG).
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

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src import picam

_latest_jpeg: Optional[bytes] = None
_latest_lock = threading.Lock()
_stop_capture = threading.Event()


def _capture_loop(picam2, jpeg_quality: int, target_period_s: float) -> None:
    global _latest_jpeg
    while not _stop_capture.is_set():
        t0 = time.monotonic()
        try:
            gray = picam.get_gray_frame(picam2)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            if ok:
                data = buf.tobytes()
                with _latest_lock:
                    _latest_jpeg = data
        except Exception as exc:
            print(f"[preview_stream] capture error: {exc}", file=sys.stderr)

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
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
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


def main() -> None:
    global _latest_jpeg

    parser = argparse.ArgumentParser(description="MJPEG live preview (Picamera2).")
    parser.add_argument("--bind", default="0.0.0.0", help="Listen address (default all interfaces).")
    parser.add_argument("--port", type=int, default=8080, help="TCP port (default 8080).")
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help=f"Frame width; height 480. Default {picam.DEFAULT_FRAME_SIZE[0]}.",
    )
    parser.add_argument("--fps", type=float, default=24.0, help="Max capture rate (default 24).")
    parser.add_argument("--quality", type=int, default=75, help="JPEG quality 1-95 (default 75).")
    args = parser.parse_args()

    frame_size = picam.normalize_resolution(args.width) if args.width is not None else picam.DEFAULT_FRAME_SIZE
    target_period = 1.0 / max(args.fps, 1.0)

    picam2 = picam.init_camera(frame_size)
    cap_thread = threading.Thread(
        target=_capture_loop,
        args=(picam2, int(args.quality), target_period),
        daemon=True,
    )
    cap_thread.start()

    print(f"Preview: open http://<this-pi>:{args.port}/  (same network as your Mac)")
    print(f"Resolution {frame_size}, ~{args.fps} fps, Ctrl+C to stop.")
    server = HTTPServer((args.bind, args.port), _MJPEGHandler)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down…")
    finally:
        _stop_capture.set()
        server.server_close()
        picam.close_camera(picam2)
        cap_thread.join(timeout=2.0)
        with _latest_lock:
            _latest_jpeg = None


if __name__ == "__main__":
    main()
