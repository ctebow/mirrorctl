"""
Raspberry Pi Camera Interface — Quadrant Position Sensor
with MJPEG browser streaming (no display required)
─────────────────────────────────────────────────────────
Designed for external import. Two public functions:

    start_camera_server()
        Initialises the camera and starts the Flask web server,
        both in background threads. Call once at startup.

    x_err, y_err = get_centroid_err()
        Captures one frame from the camera, computes quadrant fluxes
        and normalised x/y position error, updates the live web UI,
        and returns (x_err, y_err). Call this whenever you need a measurement.

Web UI is served at  http://<rpi-ip>:5000

Install deps (once):
    pip install picamera2 opencv-python flask --break-system-packages

Example usage (external file):
    from raspi_camera import start_camera_server, get_centroid_err

    start_camera_server()   # init camera + web server once

    while True:
        x_err, y_err = get_centroid_err()
        print(f"x_err={x_err:+.5f}  y_err={y_err:+.5f}")
"""

import time
import threading
import numpy as np
import cv2
from flask import Flask, Response, render_template_string, jsonify


# ──────────────────────────────────────────────────────────────────────────────
# Configuration  (edit freely)
# ──────────────────────────────────────────────────────────────────────────────
K_SCALE      = 1.0    # position scale factor; set to physical half-size for mm
FLASK_HOST   = "0.0.0.0"
FLASK_PORT   = 5000
JPEG_QUALITY = 80     # 1-100

# Camera settings
FRAME_WIDTH   = 1280
FRAME_HEIGHT  = 720
FRAME_RATE    = 30        # fps
EXPOSURE_US   = 10_000    # µs (10 ms)
ANALOGUE_GAIN = 1.0

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ──────────────────────────────────────────────────────────────────────────────
# Module-level shared state
# ──────────────────────────────────────────────────────────────────────────────
_frame_lock   = threading.Lock()
_latest_jpeg  = None
_latest_stats = {
    "frame": 0,
    "U_A": 0.0, "U_B": 0.0, "U_C": 0.0, "U_D": 0.0,
    "x": 0.0,   "y": 0.0,
}

_frame_counter  = 0
_server_started = False
_camera         = None   # Picamera2 instance, created in start_server()


# ──────────────────────────────────────────────────────────────────────────────
# Internal processing helpers
# ──────────────────────────────────────────────────────────────────────────────
def _crop_to_square(gray: np.ndarray) -> np.ndarray:
    """Centre-crop a grayscale image to the largest possible square."""
    h, w = gray.shape
    side = min(h, w)
    y0 = (h - side) // 2
    x0 = (w - side) // 2
    return gray[y0 : y0 + side, x0 : x0 + side]


def _compute_quadrants(sq: np.ndarray):
    """Return (U_A, U_B, U_C, U_D) pixel flux sums for each quadrant.

    Quadrant layout:
        A (top-left)  | B (top-right)
        ──────────────┼──────────────
        C (bot-left)  | D (bot-right)
    """
    cy, cx = sq.shape[0] // 2, sq.shape[1] // 2
    return (
        float(sq[:cy, :cx].sum()),   # A
        float(sq[:cy, cx:].sum()),   # B
        float(sq[cy:, :cx].sum()),   # C
        float(sq[cy:, cx:].sum()),   # D
    )


def _position_from_quadrants(U_A, U_B, U_C, U_D):
    """Return normalised (x, y) position from quadrant fluxes.

    x > 0 → beam is right of centre
    y > 0 → beam is above centre
    """
    total = U_A + U_B + U_C + U_D
    if total == 0:
        return 0.0, 0.0
    x = K_SCALE * ((U_B + U_D) - (U_A + U_C)) / total
    y = K_SCALE * ((U_A + U_B) - (U_C + U_D)) / total
    return x, y


# ──────────────────────────────────────────────────────────────────────────────
# Display panel builders
# ──────────────────────────────────────────────────────────────────────────────
def _build_raw_panel(square: np.ndarray, x_norm: float, y_norm: float) -> np.ndarray:
    side  = square.shape[0]
    panel = cv2.cvtColor(square.copy(), cv2.COLOR_GRAY2BGR)
    half  = side // 2

    cx_px = int(np.clip(half + x_norm / K_SCALE * half, 0, side - 1))
    cy_px = int(np.clip(half - y_norm / K_SCALE * half, 0, side - 1))

    cv2.line(panel, (half, 0),  (half, side), (70, 70, 70), 1)
    cv2.line(panel, (0, half),  (side, half), (70, 70, 70), 1)
    cv2.drawMarker(panel, (cx_px, cy_px), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)

    for label, px, py in [("A", 10, 10), ("B", half + 10, 10),
                           ("C", 10, half + 10), ("D", half + 10, half + 10)]:
        cv2.putText(panel, label, (px, py + 25), FONT, 0.9, (180, 180, 180), 1)

    cv2.putText(panel, f"Raw {side}x{side}", (8, side - 12), FONT, 0.6, (160, 160, 160), 1)
    return panel


def _build_quadrant_panel(sq_side: int, U_A, U_B, U_C, U_D) -> np.ndarray:
    half        = sq_side // 2
    flux_max    = 0.5 * half * half * 255  # 50 % of quadrant at full brightness

    def level(u):
        return int(np.clip(u / flux_max * 255, 0, 255)) if flux_max > 0 else 0

    lA, lB, lC, lD = level(U_A), level(U_B), level(U_C), level(U_D)

    panel = np.zeros((sq_side, sq_side), dtype=np.uint8)
    panel[:half, :half] = lA
    panel[:half, half:] = lB
    panel[half:, :half] = lC
    panel[half:, half:] = lD

    cv2.line(panel, (half, 0),  (half, sq_side), 128, 1)
    cv2.line(panel, (0, half),  (sq_side, half), 128, 1)

    for name, flux, ox, oy in [("A", U_A, 10, 10), ("B", U_B, half + 10, 10),
                                ("C", U_C, 10, half + 10), ("D", U_D, half + 10, half + 10)]:
        fill    = level(flux)
        txt_col = 255 if fill < 128 else 0
        frac    = flux / flux_max if flux_max > 0 else 0
        cv2.putText(panel, name,              (ox + 8, oy + 40),  FONT, 1.4,  txt_col, 2)
        cv2.putText(panel, f"{frac:.4f}",     (ox + 8, oy + 85),  FONT, 0.85, txt_col, 1)
        cv2.putText(panel, f"flux={flux:.0f}",(ox + 8, oy + 115), FONT, 0.65, txt_col, 1)

    cv2.putText(panel, "Absolute quadrant intensity",
                (8, sq_side - 12), FONT, 0.6, 180, 1)
    return cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)


def _build_combined_frame(square, U_A, U_B, U_C, U_D, x, y) -> np.ndarray:
    side     = square.shape[0]
    left     = _build_raw_panel(square, x, y)
    right    = _build_quadrant_panel(side, U_A, U_B, U_C, U_D)
    combined = np.hstack([left, right])
    cv2.putText(
        combined,
        f"x = {x:+.5f}   y = {y:+.5f}",
        (side + 10, side - 12),
        FONT, 0.85, (255, 255, 255), 2,
    )
    return combined


# ──────────────────────────────────────────────────────────────────────────────
# Flask app (module-level singleton)
# ──────────────────────────────────────────────────────────────────────────────
_app = Flask(__name__)

_PAGE = """<!DOCTYPE html>
<html>
<head>
  <title>RPi Quadrant Sensor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0e0e0e; color: #e0e0e0;
      font-family: 'Courier New', monospace;
      display: flex; flex-direction: column; align-items: center;
      padding: 20px; gap: 16px;
    }
    h2 { color: #7dd3fc; letter-spacing: 3px; font-size: 1.1rem; margin-top: 4px; }
    #feed-wrap {
      width: 100%; max-width: 1440px;
      border: 1px solid #2a2a2a; border-radius: 6px; overflow: hidden;
      background: #000;
    }
    #feed-wrap img { width: 100%; display: block; }
    #stats {
      width: 100%; max-width: 1440px;
      background: #161616; border: 1px solid #2a2a2a; border-radius: 6px;
      padding: 14px 20px; display: grid;
      grid-template-columns: repeat(4, 1fr); gap: 10px 24px;
    }
    .stat { display: flex; flex-direction: column; gap: 2px; }
    .stat-label { font-size: 0.65rem; color: #7dd3fc; text-transform: uppercase; letter-spacing: 1px; }
    .stat-value { font-size: 1.1rem; color: #f0f0f0; }
    .pos-row {
      grid-column: 1 / -1;
      display: flex; gap: 40px; border-top: 1px solid #2a2a2a;
      padding-top: 10px; margin-top: 4px;
    }
    .pos-val { font-size: 1.3rem; color: #86efac; }
    #waiting {
      color: #555; font-size: 0.85rem; padding: 60px 0; text-align: center;
    }
  </style>
</head>
<body>
  <h2>&#9632; RPi Camera QPD Analog</h2>
  <div id="feed-wrap">
    <img src="/video_feed" alt="stream" onerror="this.style.display='none';document.getElementById('waiting').style.display='block'">
    <div id="waiting" style="display:none">Waiting for first frame…</div>
  </div>
  <div id="stats">
    <div class="stat"><span class="stat-label">Frame</span><span class="stat-value" id="s-frame">—</span></div>
    <div class="stat"><span class="stat-label">U_A (top-left)</span><span class="stat-value" id="s-ua">—</span></div>
    <div class="stat"><span class="stat-label">U_B (top-right)</span><span class="stat-value" id="s-ub">—</span></div>
    <div class="stat"><span class="stat-label">U_C (bot-left)</span><span class="stat-value" id="s-uc">—</span></div>
    <div class="stat"><span class="stat-label">U_D (bot-right)</span><span class="stat-value" id="s-ud">—</span></div>
    <div class="pos-row">
      <div class="stat"><span class="stat-label">x (normalised)</span><span class="pos-val" id="s-x">—</span></div>
      <div class="stat"><span class="stat-label">y (normalised)</span><span class="pos-val" id="s-y">—</span></div>
    </div>
  </div>
  <script>
    function fmt(v) { return (v >= 0 ? "+" : "") + v.toFixed(5); }
    async function poll() {
      try {
        const d = await (await fetch("/data")).json();
        if (d.frame === 0) return;
        document.getElementById("s-frame").textContent = d.frame;
        document.getElementById("s-ua").textContent    = d.U_A.toFixed(0);
        document.getElementById("s-ub").textContent    = d.U_B.toFixed(0);
        document.getElementById("s-uc").textContent    = d.U_C.toFixed(0);
        document.getElementById("s-ud").textContent    = d.U_D.toFixed(0);
        document.getElementById("s-x").textContent     = fmt(d.x);
        document.getElementById("s-y").textContent     = fmt(d.y);
      } catch(e) {}
    }
    setInterval(poll, 150);
    poll();
  </script>
</body>
</html>"""


@_app.route("/")
def _index():
    return render_template_string(_PAGE)


@_app.route("/video_feed")
def _video_feed():
    """MJPEG stream — delivers whatever the latest processed frame is."""
    def generate():
        last_sent = None
        while True:
            with _frame_lock:
                frame = _latest_jpeg
            if frame and frame is not last_sent:
                last_sent = frame
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame +
                    b"\r\n"
                )
            else:
                time.sleep(0.01)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@_app.route("/data")
def _data():
    with _frame_lock:
        return jsonify(_latest_stats)


# ──────────────────────────────────────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────────────────────────────────────
def start_camera_server(host: str = FLASK_HOST, port: int = FLASK_PORT) -> None:
    """Initialise the camera and start the Flask web server.

    Call once at the start of your program. Returns immediately;
    the camera is ready and the server runs in background threads.

    Args:
        host: Interface to bind (default "0.0.0.0" = all interfaces).
        port: TCP port (default 5000).
    """
    global _server_started, _camera
    if _server_started:
        return
    _server_started = True

    # ── Init camera ──────────────────────────────────────────────────────────
    from picamera2 import Picamera2
    _camera = Picamera2()
    fd = int(1_000_000 / FRAME_RATE)
    _camera.configure(_camera.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "YUV420"},
        controls={
            "AeEnable":            False,
            "AwbEnable":           False,
            "ExposureTime":        EXPOSURE_US,
            "AnalogueGain":        ANALOGUE_GAIN,
            "FrameDurationLimits": (fd, fd),
        },
    ))
    _camera.start()
    time.sleep(0.5)   # let AEC/AWB settle even though we've disabled them

    meta = _camera.capture_metadata()
    print("── Camera settings ─────────────────────────")
    for k in ("ExposureTime", "AnalogueGain", "FrameDuration"):
        print(f"  {k:22s}: {meta.get(k, 'n/a')}")
    print("────────────────────────────────────────────")

    # ── Start Flask ──────────────────────────────────────────────────────────
    def _run():
        import logging
        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        _app.run(host=host, port=port, threaded=True, use_reloader=False)

    threading.Thread(target=_run, daemon=True, name="qpd-flask").start()
    print(f"[QPD] Web server started → http://<rpi-ip>:{port}")


def get_centroid_err() -> tuple[float, float]:
    """Capture one frame and return the normalised (x_err, y_err) beam position error.

    The camera must have been initialised by calling start_camera_server() first.
    Also updates the live web UI (video feed + stats panel).

    Returns:
        (x_err, y_err): Normalised position error values in [-K_SCALE, +K_SCALE].
                        x_err > 0 → beam right of centre
                        y_err > 0 → beam above centre

    Raises:
        RuntimeError: If start_camera_server() has not been called yet.
    """
    global _frame_counter, _latest_jpeg, _latest_stats

    if _camera is None:
        raise RuntimeError("[QPD] Camera not initialised — call start_camera_server() first.")

    yuv    = _camera.capture_array("main")
    gray   = yuv[:FRAME_HEIGHT, :FRAME_WIDTH].copy()   # Y-plane from YUV420

    square = _crop_to_square(gray)
    U_A, U_B, U_C, U_D = _compute_quadrants(square)
    x, y = _position_from_quadrants(U_A, U_B, U_C, U_D)

    _frame_counter += 1

    combined = _build_combined_frame(square, U_A, U_B, U_C, U_D, x, y)
    ok, jpeg_buf = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

    if ok:
        with _frame_lock:
            _latest_jpeg  = jpeg_buf.tobytes()
            _latest_stats = {
                "frame": _frame_counter,
                "U_A": U_A, "U_B": U_B, "U_C": U_C, "U_D": U_D,
                "x": round(x, 6), "y": round(y, 6),
            }

    return x, y


# ──────────────────────────────────────────────────────────────────────────────
# Entry point: run standalone with live camera (original behaviour)
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    start_camera_server()

    header = (f"{'Frame':>7}  {'U_A':>12}  {'U_B':>12}  "
              f"{'U_C':>12}  {'U_D':>12}  {'x_err':>10}  {'y_err':>10}")
    print(header)
    print("─" * len(header))

    try:
        while True:
            x_err, y_err = get_centroid_err()
            s = _latest_stats
            print(
                f"{s['frame']:>7}  {s['U_A']:>12.0f}  {s['U_B']:>12.0f}  "
                f"{s['U_C']:>12.0f}  {s['U_D']:>12.0f}  "
                f"{x_err:>+10.5f}  {y_err:>+10.5f}",
                flush=True,
            )
    except KeyboardInterrupt:
        if _camera:
            _camera.stop()
        print(f"\n[QPD] {_frame_counter} frames processed.")