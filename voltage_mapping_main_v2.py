"""
Interactive voltage mapping entrypoint with prompt or params-file inputs.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any, Optional

import cv2

from src import FSM, centroiding, picam
from src.constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS
from src.exceptions import CalibrationError, HardwareUnavailable, UnsafeVoltageRequest
from services import CalibrationService, MappingService, MappingSweepParams
from config import calibrate_camera_interactive, calibrate_picam


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CAL = SCRIPT_DIR / "config" / "camera_params.npz"
DEFAULT_OUT_DIR = SCRIPT_DIR / "runs" / "mapping"
DEFAULT_OUTFILE = DEFAULT_OUT_DIR / "voltage_mapping_out.csv"
PARAMS_FILENAME = "voltage_mapping_params.json"


@dataclass
class RunConfig:
    mode: str
    outfile: Path
    num_frames: int = 5
    settling_time: float = 0.1
    resolution: int = 640
    roi: int = 50
    axis: str = "x"
    start_vdiff: float = 0.0
    end_vdiff: float = 150.0
    step_size: float = 1.0
    manual_x: float = 0.0
    manual_y: float = 0.0
    grid_start_x: float = 0.0
    grid_start_y: float = 0.0
    grid_end_x: float = 150.0
    grid_end_y: float = 150.0
    grid_step_size: float = 1.0
    no_calib: bool = False
    calibration_path: Path = DEFAULT_CAL
    distance_to_board: Optional[float] = None
    preview_bind: str = "0.0.0.0"
    preview_port: int = 8080
    preview_fps: float = 24.0
    preview_quality: int = 75


def _norm_mode(raw: str) -> str:
    v = raw.strip().lower()
    aliases = {
        "preview-cam": "preview-cam",
        "preview": "preview-cam",
        "test-cam": "preview-cam",
        "man": "man",
        "manual": "man",
        "sweep": "sweep",
        "auto": "sweep",
        "grid": "grid",
    }
    if v not in aliases:
        raise ValueError(f"Invalid mode: {raw}")
    return aliases[v]


def _prompt_with_default(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return raw if raw else default


def _prompt_bool(prompt: str, default: bool) -> bool:
    default_str = "y" if default else "n"
    while True:
        v = _prompt_with_default(f"{prompt} (y/n)", default_str).strip().lower()
        if v in {"y", "yes"}:
            return True
        if v in {"n", "no"}:
            return False
        print("Please answer y or n.", file=sys.stderr)


def _prompt_int(prompt: str, default: int, minimum: Optional[int] = None) -> int:
    while True:
        raw = _prompt_with_default(prompt, str(default))
        try:
            value = int(raw)
            if minimum is not None and value < minimum:
                raise ValueError
            return value
        except ValueError:
            msg = f"Please enter an integer >= {minimum}." if minimum is not None else "Please enter an integer."
            print(msg, file=sys.stderr)


def _prompt_float(prompt: str, default: float, minimum: Optional[float] = None) -> float:
    while True:
        raw = _prompt_with_default(prompt, str(default))
        try:
            value = float(raw)
            if minimum is not None and value < minimum:
                raise ValueError
            return value
        except ValueError:
            msg = f"Please enter a number >= {minimum}." if minimum is not None else "Please enter a number."
            print(msg, file=sys.stderr)


def _load_params_file() -> dict[str, Any]:
    p = SCRIPT_DIR / PARAMS_FILENAME
    if not p.is_file():
        raise FileNotFoundError(f"Could not find {PARAMS_FILENAME} in {SCRIPT_DIR}")
    with open(p, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Params file must contain a top-level object.")
    return data


def _config_from_params(data: dict[str, Any]) -> RunConfig:
    mode = _norm_mode(str(data.get("mode", "man")))
    mode_block = data.get(mode.replace("-", "_"), {})
    if not isinstance(mode_block, dict):
        raise ValueError(f"Mode block for {mode} must be an object.")

    cfg = RunConfig(
        mode=mode,
        outfile=Path(str(data.get("outfile", str(DEFAULT_OUTFILE)))),
        num_frames=int(data.get("num_frames", 5)),
        settling_time=float(data.get("settling_time", 0.1)),
        resolution=int(data.get("resolution", 640)),
        roi=int(data.get("roi", 50)),
        no_calib=bool(data.get("no_calib", False)),
        calibration_path=Path(str(data.get("calibration_path", str(DEFAULT_CAL)))),
        distance_to_board=(
            float(data["distance_to_board"]) if data.get("distance_to_board") is not None else None
        ),
        preview_bind=str(data.get("preview_bind", "0.0.0.0")),
        preview_port=int(data.get("preview_port", 8080)),
        preview_fps=float(data.get("preview_fps", 24.0)),
        preview_quality=int(data.get("preview_quality", 75)),
    )

    if mode == "man":
        cfg.manual_x = float(mode_block.get("vdiff_x", 0.0))
        cfg.manual_y = float(mode_block.get("vdiff_y", 0.0))
    elif mode == "sweep":
        cfg.axis = str(mode_block.get("axis", "x")).lower()
        cfg.start_vdiff = float(mode_block.get("start_vdiff", 0.0))
        cfg.end_vdiff = float(mode_block.get("end_vdiff", 150.0))
        cfg.step_size = float(mode_block.get("step_size", 1.0))
    elif mode == "grid":
        start = mode_block.get("start_corner", [0.0, 0.0])
        end = mode_block.get("end_corner", [150.0, 150.0])
        if not (isinstance(start, list) and len(start) == 2 and isinstance(end, list) and len(end) == 2):
            raise ValueError("grid.start_corner and grid.end_corner must be [x, y].")
        cfg.grid_start_x = float(start[0])
        cfg.grid_start_y = float(start[1])
        cfg.grid_end_x = float(end[0])
        cfg.grid_end_y = float(end[1])
        cfg.grid_step_size = float(mode_block.get("step_size", 1.0))
    return cfg


def _config_from_prompts() -> RunConfig:
    mode = _norm_mode(_prompt_with_default("Mode (preview-cam | man | sweep | grid)", "man"))
    cfg = RunConfig(
        mode=mode,
        outfile=Path(_prompt_with_default("Output CSV path", str(DEFAULT_OUTFILE))),
        num_frames=_prompt_int("Frames per measurement", 5, minimum=1),
        settling_time=_prompt_float("Settling time (seconds)", 0.1, minimum=0.0),
        resolution=_prompt_int("Camera width (height fixed to 480)", 640, minimum=1),
        roi=_prompt_int("Centroid ROI half-size", 50, minimum=1),
        no_calib=_prompt_bool("Skip calibration file (raw pixels only)", False),
        calibration_path=Path(_prompt_with_default("Calibration file path", str(DEFAULT_CAL))),
        distance_to_board=(
            _prompt_float("Distance to board (same units as x_mm/y_mm; blank disables)", 0.0, minimum=0.0)
            if _prompt_bool("Compute angular displacement columns?", False)
            else None
        ),
        preview_bind=_prompt_with_default("Preview bind address", "0.0.0.0"),
        preview_port=_prompt_int("Preview port", 8080, minimum=1),
        preview_fps=_prompt_float("Preview FPS", 24.0, minimum=1.0),
        preview_quality=_prompt_int("Preview JPEG quality", 75, minimum=1),
    )

    if mode == "man":
        cfg.manual_x = _prompt_float("Initial vdiff x", 0.0)
        cfg.manual_y = _prompt_float("Initial vdiff y", 0.0)
    elif mode == "sweep":
        cfg.axis = _prompt_with_default("Sweep axis (x|y)", "x").strip().lower()
        cfg.start_vdiff = _prompt_float("Sweep start vdiff", 0.0)
        cfg.end_vdiff = _prompt_float("Sweep end vdiff", 150.0)
        cfg.step_size = _prompt_float("Sweep step size", 1.0)
    elif mode == "grid":
        cfg.grid_start_x = _prompt_float("Grid start corner x", 0.0)
        cfg.grid_start_y = _prompt_float("Grid start corner y", 0.0)
        cfg.grid_end_x = _prompt_float("Grid end corner x", 150.0)
        cfg.grid_end_y = _prompt_float("Grid end corner y", 150.0)
        cfg.grid_step_size = _prompt_float("Grid step size", 1.0)
    return cfg


def _validate_cfg(cfg: RunConfig) -> None:
    if cfg.mode not in {"preview-cam", "man", "sweep", "grid"}:
        raise ValueError(f"Unsupported mode: {cfg.mode}")
    if cfg.mode == "sweep":
        if cfg.axis not in {"x", "y"}:
            raise ValueError("sweep.axis must be x or y")
        if abs(cfg.step_size) <= 1e-9:
            raise ValueError("sweep.step_size must be non-zero")
    if cfg.mode == "grid" and abs(cfg.grid_step_size) <= 1e-9:
        raise ValueError("grid.step_size must be non-zero")
    if cfg.distance_to_board is not None and cfg.distance_to_board <= 0:
        raise ValueError("distance_to_board must be positive when provided")


def _open_camera(resolution: int):
    try:
        return picam.init_camera(resolution)
    except Exception as exc:
        raise HardwareUnavailable(f"Camera failed: {exc}") from exc


def _capture_homography_ref_image(width: int, out_path: Path) -> Path:
    cam = None
    try:
        cam = _open_camera(width)
        gray = picam.get_gray_frame(cam)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(out_path), bgr):
            raise RuntimeError(f"Failed to write homography image to {out_path}")
        return out_path
    finally:
        picam.close_camera(cam)


def _run_calibration_startup() -> None:
    if not _prompt_bool("Run camera calibration startup flow now?", True):
        return
    print("\nCalibration options:")
    print("  1) Full interactive calibration (lens + homography capture)")
    print("  2) Update homography only (capture one fresh board image now)")
    print("  3) Skip calibration")
    choice = _prompt_with_default("Select option", "1").strip()

    if choice == "3":
        return

    if choice == "1":
        rc = calibrate_camera_interactive.main()
        if rc != 0:
            raise RuntimeError(f"Interactive calibration failed with exit code {rc}")
        return

    if choice == "2":
        width = _prompt_int("Homography capture width", 640, minimum=1)
        ref_path = SCRIPT_DIR / "config" / "homography_ref_latest.jpg"
        ref = _capture_homography_ref_image(width, ref_path)
        board, _, detector = calibrate_picam.make_charuco_board(
            calibrate_picam.SQUARES_X,
            calibrate_picam.SQUARES_Y,
            calibrate_picam.SQUARE_LENGTH,
            calibrate_picam.MARKER_LENGTH,
            calibrate_picam.DICT_TYPE,
            legacy_pattern=True,
        )
        calibrate_picam.update_homography_only(ref, board, detector)
        return

    raise ValueError("Invalid calibration option.")


_latest_jpeg: Optional[bytes] = None
_latest_lock = threading.Lock()
_stop_preview = threading.Event()


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
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if ok:
                with _latest_lock:
                    _latest_jpeg = buf.tobytes()
        except Exception as exc:
            print(f"[preview] capture error: {exc}", file=sys.stderr)
        dt = time.monotonic() - t0
        to_sleep = period_s - dt
        if to_sleep > 0:
            time.sleep(to_sleep)


def _run_preview_mode(cfg: RunConfig) -> int:
    cam = None
    server: Optional[HTTPServer] = None
    server_thread: Optional[threading.Thread] = None
    cap_thread: Optional[threading.Thread] = None
    try:
        _stop_preview.clear()
        cam = _open_camera(cfg.resolution)
        cap_thread = threading.Thread(
            target=_preview_capture_loop,
            args=(cam, int(cfg.preview_quality), 1.0 / max(cfg.preview_fps, 1.0)),
            daemon=True,
        )
        cap_thread.start()
        server = HTTPServer((cfg.preview_bind, cfg.preview_port), _PreviewHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print("Preview mode running.")
        print(f"Open on Mac (HTTPS link format): https://<this-pi>:{cfg.preview_port}/")
        print(f"Direct stream endpoint: http://<this-pi>:{cfg.preview_port}/")
        print("Press ENTER to stop preview...")
        input()
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


def _axis_points(start: float, end: float, step: float) -> list[float]:
    eps = 1e-9
    if abs(end - start) <= eps:
        return [float(start)]
    if abs(step) <= eps:
        raise ValueError("Step cannot be zero when start != end.")
    delta = end - start
    if (delta > 0 and step < 0) or (delta < 0 and step > 0):
        raise ValueError("Step sign does not move start toward end.")
    vals: list[float] = []
    curr = float(start)
    while True:
        vals.append(curr)
        nxt = curr + step
        if (step > 0 and nxt >= end - eps) or (step < 0 and nxt <= end + eps):
            if abs(vals[-1] - end) > eps:
                vals.append(float(end))
            break
        curr = nxt
    return vals


def _augment_rows_with_angles(
    rows: list[list[Any]],
    header: list[str],
    distance_to_board: Optional[float],
) -> tuple[list[list[Any]], list[str]]:
    if distance_to_board is None:
        return rows, header
    if "x_mm" not in header or "y_mm" not in header:
        print(
            "distance_to_board provided, but CSV rows do not include x_mm/y_mm. "
            "Skipping angular displacement columns.",
            file=sys.stderr,
        )
        return rows, header
    x_idx = header.index("x_mm")
    y_idx = header.index("y_mm")
    out_header = [*header, "theta_x_deg", "theta_y_deg"]
    out_rows: list[list[Any]] = []
    for row in rows:
        x_mm = float(row[x_idx])
        y_mm = float(row[y_idx])
        theta_x = math.degrees(math.atan2(x_mm, distance_to_board))
        theta_y = math.degrees(math.atan2(y_mm, distance_to_board))
        out_rows.append([*row, theta_x, theta_y])
    return out_rows, out_header


def _run_man_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    rows: list[list[Any]] = []
    print("[MAN] Type 'q' to stop and write CSV.")
    print(f"[MAN] initial command vdiff=({cfg.manual_x}, {cfg.manual_y})")
    try:
        fsm.set_vdiff(cfg.manual_x, cfg.manual_y)
        while True:
            usr = input("Input vdiffx vdiffy (or q): ").strip()
            if usr.lower() in {"q", "quit", "exit"}:
                break
            parts = usr.split()
            if len(parts) != 2:
                print("Bad coords. Expected: <x y>", file=sys.stderr)
                continue
            try:
                vx_cmd = float(parts[0])
                vy_cmd = float(parts[1])
                fsm.set_vdiff(vx_cmd, vy_cmd)
            except ValueError:
                print("Bad coords. Expected: <x y>", file=sys.stderr)
                continue
            except UnsafeVoltageRequest as exc:
                print(f"Unsafe: {exc}", file=sys.stderr)
                continue
            centroid = mapper.capture_centroid_averages(cam, cfg.num_frames, cfg.roi, calib)
            vx, vy = fsm.get_voltages()
            rows.append([vx, vy, *centroid])
    except KeyboardInterrupt:
        print("\nInterrupted; writing CSV.")
    header = mapper.csv_header(calib)
    rows, header = _augment_rows_with_angles(rows, header, cfg.distance_to_board)
    mapper.write_csv(cfg.outfile, rows, header, mode="w")
    print(f"Wrote {cfg.outfile} ({len(rows)} rows)")
    return 0


def _run_sweep_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    params = MappingSweepParams(
        num_frames=cfg.num_frames,
        settling_time=cfg.settling_time,
        axis=cfg.axis,
        step_size=cfg.step_size,
        start=cfg.start_vdiff,
        end=cfg.end_vdiff,
        roi=cfg.roi,
    )
    rows = mapper.run_auto_sweep(fsm, cam, params, calib)
    header = mapper.csv_header(calib)
    rows, header = _augment_rows_with_angles(rows, header, cfg.distance_to_board)
    mapper.write_csv(cfg.outfile, rows, header, mode="w")
    print(f"Wrote {cfg.outfile} ({len(rows)} rows)")
    return 0


def _run_grid_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    step_x = cfg.grid_step_size if cfg.grid_end_x >= cfg.grid_start_x else -cfg.grid_step_size
    step_y = cfg.grid_step_size if cfg.grid_end_y >= cfg.grid_start_y else -cfg.grid_step_size
    xs = _axis_points(cfg.grid_start_x, cfg.grid_end_x, step_x)
    ys = _axis_points(cfg.grid_start_y, cfg.grid_end_y, step_y)

    rows: list[list[Any]] = []
    for yi, yv in enumerate(ys):
        x_seq = xs if yi % 2 == 0 else list(reversed(xs))
        for xv in x_seq:
            fsm.set_vdiff(xv, yv)
            centroid = mapper.capture_centroid_averages(cam, cfg.num_frames, cfg.roi, calib)
            vx, vy = fsm.get_voltages()
            rows.append([vx, vy, *centroid])
            time.sleep(cfg.settling_time)

    header = mapper.csv_header(calib)
    rows, header = _augment_rows_with_angles(rows, header, cfg.distance_to_board)
    mapper.write_csv(cfg.outfile, rows, header, mode="w")
    print(f"Wrote {cfg.outfile} ({len(rows)} rows)")
    return 0


def _run_measurement_mode(cfg: RunConfig) -> int:
    mapper = MappingService()
    calib: centroiding.CameraCalibration | None = None
    cam = None
    fsm = FSM()
    try:
        cam = _open_camera(cfg.resolution)
        if not cfg.no_calib:
            calib = CalibrationService.load(cfg.calibration_path)
        result = fsm.begin_interactive()
        if not result.ok:
            print("FSM failed to start or was aborted.", file=sys.stderr)
            return 1
        if cfg.mode == "man":
            return _run_man_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "sweep":
            return _run_sweep_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "grid":
            return _run_grid_mode(cfg, mapper, fsm, cam, calib)
        raise ValueError(f"Unsupported measurement mode: {cfg.mode}")
    finally:
        try:
            fsm.close()
        except Exception:
            pass
        picam.close_camera(cam)


def main() -> int:
    print("--- Voltage Mapping Main v2 ---")
    try:
        use_params = _prompt_bool(f"Use parameters file ({PARAMS_FILENAME})?", False)
        if use_params:
            cfg = _config_from_params(_load_params_file())
        else:
            cfg = _config_from_prompts()
        _validate_cfg(cfg)

        _run_calibration_startup()

        if cfg.mode == "preview-cam":
            return _run_preview_mode(cfg)
        return _run_measurement_mode(cfg)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except (ValueError, CalibrationError, UnsafeVoltageRequest) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except HardwareUnavailable as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
