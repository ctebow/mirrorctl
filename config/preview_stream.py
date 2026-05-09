"""
Live MJPEG preview server for headless Pi → view in a browser on your Mac.

No recording; use this only to frame/focus, then use get_calib_photos.py for saves.

On Pi:
  python3 config/preview_stream.py --bind 0.0.0.0 --port 8080

On Mac browser:
  http://<pi-ip>:8080/

Stop with Ctrl+C on the Pi. Same Picamera2 config as mapping (RGB888 → gray → BGR JPEG).
Pass --max-res to use the sensor's full native resolution (slower fps recommended).
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


def get_max_resolution(picam2) -> tuple[int, int]:
    """Return the sensor's maximum resolution by inspecting available modes."""
    picam2.stop()
    modes = picam2.sensor_modes
    return max(modes, key=lambda m: m["size"][0] * m["size"][1])["size"]


def _draw_grid(img: cv2.Mat, grid_size: int, color: tuple[int, int, int] = (0, 0, 255), thickness: int = 1) -> None:
    """Draw an evenly spaced grid centered at the image midpoint.

    The x=0 (vertical center) and y=0 (horizontal center) axes are drawn in
    blue; all other grid lines are drawn in red.
    """
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    axis_color = (0, 255, 0)  # blue (BGR)

    # Vertical lines, stepping out from center
    x = cx % grid_size  # leftmost line position
    while x < w:
        c = axis_color if x == cx else color
        cv2.line(img, (x, 0), (x, h), c, thickness)
        x += grid_size

    # Horizontal lines, stepping out from center
    y = cy % grid_size  # topmost line position
    while y < h:
        c = axis_color if y == cy else color
        cv2.line(img, (0, y), (w, y), c, thickness)
        y += grid_size


def _capture_loop(picam2, jpeg_quality: int, target_period_s: float, grid_size: int) -> None:
    global _latest_jpeg
    while not _stop_capture.is_set():
        t0 = time.monotonic()
        try:
            gray = picam.get_gray_frame(picam2)
            gray = cv2.flip(gray, 0)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if grid_size > 0:
                _draw_grid(bgr, grid_size)
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
        help=f"Frame width. Default {picam.DEFAULT_FRAME_SIZE[0]}. Ignored if --max-res is set.",
    )
    parser.add_argument(
        "--max-res",
        action="store_true",
        help="Use the sensor's maximum native resolution. Overrides --width.",
    )
    parser.add_argument("--fps", type=float, default=24.0, help="Max capture rate (default 24). Consider 5-10 with --max-res.")
    parser.add_argument("--quality", type=int, default=75, help="JPEG quality 1-95 (default 75).")
    parser.add_argument("--grid", type=int, default=100, metavar="PX", help="Grid square size in pixels (default 100). Set to 0 to disable.")
    args = parser.parse_args()

    if args.max_res:
        # Init with default size just long enough to query sensor modes, then re-init at full res.
        print("Querying sensor for maximum resolution…")
        _tmp = picam.init_camera(picam.DEFAULT_FRAME_SIZE)
        frame_size = get_max_resolution(_tmp)
        picam.close_camera(_tmp)
        print(f"Sensor max resolution: {frame_size[0]}x{frame_size[1]}")
    else:
        # Derive height from sensor aspect ratio so we don't get a squished widescreen frame.
        # This applies both when --width is given and when falling back to the default width.
        print("Querying sensor aspect ratio…")
        _tmp = picam.init_camera(picam.DEFAULT_FRAME_SIZE)
        max_w, max_h = get_max_resolution(_tmp)
        picam.close_camera(_tmp)
        aspect = max_h / max_w
        width = args.width if args.width is not None else picam.DEFAULT_FRAME_SIZE[0]
        frame_size = (width, int(round(width * aspect)))
        print(f"Sensor aspect ratio {max_w}x{max_h} → frame size {frame_size[0]}x{frame_size[1]}")

    target_period = 1.0 / max(args.fps, 1.0)

    picam2 = picam.init_camera(frame_size)
    picam2.set_controls({
        "AeEnable": False,
        "ExposureTime": 200,  # microseconds — adjust this
    })

    cap_thread = threading.Thread(
        target=_capture_loop,
        args=(picam2, int(args.quality), target_period, args.grid),
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