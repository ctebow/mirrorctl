"""
Interactive voltage mapping entrypoint with prompt or params-file inputs.

Modes: preview-cam, man, sweep, grid, verify, verify-sweep, verify-grid.

verify: After fitting polynomials from prior mapping CSV(s), enter manual vdiff pairs;
live centroid → actual x_mm/y_mm vs model expectation; results printed and written to outfile.

verify-sweep: Same polynomial options as verify, but runs an automatic sweep (axis/start/end/step
like sweep mode). CSV is the sweep CSV plus expected_x, expected_y, difference_x, difference_y
(board-plane mm; differences are actual − expected, matching column x_mm / y_mm).

verify-grid: Same as verify-sweep, but traverses a full grid (start_corner, end_corner, step_size)
like grid mode (serpentine x-order per row).
"""
from __future__ import annotations

import csv
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from collections.abc import Callable
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

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


def _resolve_script_path(p: Path) -> Path:
    return p if p.is_absolute() else (SCRIPT_DIR / p)


def _norm_csv_header(name: str) -> str:
    return name.strip().lower().lstrip("\ufeff")


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
    # verify: polynomial model vs live centroid (board-plane mm)
    verify_fit: str = "grid"
    verify_grid_csv: Optional[Path] = None
    verify_sweep_csv_x: Optional[Path] = None
    verify_sweep_csv_y: Optional[Path] = None
    verify_poly_degree: int = 2


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
        "verify": "verify",
        "verify-sweep": "verify-sweep",
        "verify-grid": "verify-grid",
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
    elif mode == "verify":
        cfg.verify_fit = str(mode_block.get("fit", "grid")).strip().lower()
        g = mode_block.get("grid_csv")
        cfg.verify_grid_csv = Path(str(g)) if g else None
        sx = mode_block.get("sweep_csv_x")
        sy = mode_block.get("sweep_csv_y")
        cfg.verify_sweep_csv_x = Path(str(sx)) if sx else None
        cfg.verify_sweep_csv_y = Path(str(sy)) if sy else None
        cfg.verify_poly_degree = int(mode_block.get("poly_degree", 2))
    elif mode == "verify-sweep":
        cfg.axis = str(mode_block.get("axis", "x")).lower()
        cfg.start_vdiff = float(mode_block.get("start_vdiff", 0.0))
        cfg.end_vdiff = float(mode_block.get("end_vdiff", 150.0))
        cfg.step_size = float(mode_block.get("step_size", 1.0))
        cfg.verify_fit = str(mode_block.get("fit", "grid")).strip().lower()
        g = mode_block.get("grid_csv")
        cfg.verify_grid_csv = Path(str(g)) if g else None
        sx = mode_block.get("sweep_csv_x")
        sy = mode_block.get("sweep_csv_y")
        cfg.verify_sweep_csv_x = Path(str(sx)) if sx else None
        cfg.verify_sweep_csv_y = Path(str(sy)) if sy else None
        cfg.verify_poly_degree = int(mode_block.get("poly_degree", 2))
    elif mode == "verify-grid":
        start = mode_block.get("start_corner", [0.0, 0.0])
        end = mode_block.get("end_corner", [150.0, 150.0])
        if not (isinstance(start, list) and len(start) == 2 and isinstance(end, list) and len(end) == 2):
            raise ValueError("verify_grid.start_corner and end_corner must be [x, y].")
        cfg.grid_start_x = float(start[0])
        cfg.grid_start_y = float(start[1])
        cfg.grid_end_x = float(end[0])
        cfg.grid_end_y = float(end[1])
        cfg.grid_step_size = float(mode_block.get("step_size", 1.0))
        cfg.verify_fit = str(mode_block.get("fit", "grid")).strip().lower()
        g = mode_block.get("grid_csv")
        cfg.verify_grid_csv = Path(str(g)) if g else None
        sx = mode_block.get("sweep_csv_x")
        sy = mode_block.get("sweep_csv_y")
        cfg.verify_sweep_csv_x = Path(str(sx)) if sx else None
        cfg.verify_sweep_csv_y = Path(str(sy)) if sy else None
        cfg.verify_poly_degree = int(mode_block.get("poly_degree", 2))
    return cfg


def _config_from_prompts() -> RunConfig:
    mode = _norm_mode(
        _prompt_with_default(
            "Mode (preview-cam | man | sweep | grid | verify | verify-sweep | verify-grid)", "man"
        )
    )
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
    elif mode == "verify":
        cfg.verify_fit = _prompt_with_default("Verify fit source (grid | sweep)", "grid").strip().lower()
        cfg.verify_poly_degree = _prompt_int("Polynomial degree", 2, minimum=1)
        if cfg.verify_fit == "grid":
            cfg.verify_grid_csv = Path(
                _prompt_with_default("Grid mapping CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_x = None
            cfg.verify_sweep_csv_y = None
        elif cfg.verify_fit == "sweep":
            cfg.verify_sweep_csv_x = Path(
                _prompt_with_default("X-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_y = Path(
                _prompt_with_default("Y-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_grid_csv = None
        else:
            raise ValueError("verify fit must be 'grid' or 'sweep'")
    elif mode == "verify-sweep":
        cfg.axis = _prompt_with_default("Sweep axis (x|y)", "x").strip().lower()
        cfg.start_vdiff = _prompt_float("Sweep start vdiff", 0.0)
        cfg.end_vdiff = _prompt_float("Sweep end vdiff", 150.0)
        cfg.step_size = _prompt_float("Sweep step size", 1.0)
        cfg.verify_fit = _prompt_with_default("Verify fit source (grid | sweep)", "grid").strip().lower()
        cfg.verify_poly_degree = _prompt_int("Polynomial degree", 2, minimum=1)
        if cfg.verify_fit == "grid":
            cfg.verify_grid_csv = Path(
                _prompt_with_default("Grid mapping CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_x = None
            cfg.verify_sweep_csv_y = None
        elif cfg.verify_fit == "sweep":
            cfg.verify_sweep_csv_x = Path(
                _prompt_with_default("X-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_y = Path(
                _prompt_with_default("Y-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_grid_csv = None
        else:
            raise ValueError("verify fit must be 'grid' or 'sweep'")
    elif mode == "verify-grid":
        cfg.grid_start_x = _prompt_float("Grid start corner x", 0.0)
        cfg.grid_start_y = _prompt_float("Grid start corner y", 0.0)
        cfg.grid_end_x = _prompt_float("Grid end corner x", 150.0)
        cfg.grid_end_y = _prompt_float("Grid end corner y", 150.0)
        cfg.grid_step_size = _prompt_float("Grid step size", 1.0)
        cfg.verify_fit = _prompt_with_default("Verify fit source (grid | sweep)", "grid").strip().lower()
        cfg.verify_poly_degree = _prompt_int("Polynomial degree", 2, minimum=1)
        if cfg.verify_fit == "grid":
            cfg.verify_grid_csv = Path(
                _prompt_with_default("Grid mapping CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_x = None
            cfg.verify_sweep_csv_y = None
        elif cfg.verify_fit == "sweep":
            cfg.verify_sweep_csv_x = Path(
                _prompt_with_default("X-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_sweep_csv_y = Path(
                _prompt_with_default("Y-axis sweep CSV path", str(DEFAULT_OUTFILE))
            )
            cfg.verify_grid_csv = None
        else:
            raise ValueError("verify fit must be 'grid' or 'sweep'")
    return cfg


def _validate_cfg(cfg: RunConfig) -> None:
    if cfg.mode not in {"preview-cam", "man", "sweep", "grid", "verify", "verify-sweep", "verify-grid"}:
        raise ValueError(f"Unsupported mode: {cfg.mode}")
    if cfg.mode in {"sweep", "verify-sweep"}:
        if cfg.axis not in {"x", "y"}:
            raise ValueError("sweep.axis must be x or y")
        if abs(cfg.step_size) <= 1e-9:
            raise ValueError("sweep.step_size must be non-zero")
    if cfg.mode in {"grid", "verify-grid"} and abs(cfg.grid_step_size) <= 1e-9:
        raise ValueError("grid.step_size must be non-zero")
    if cfg.distance_to_board is not None and cfg.distance_to_board <= 0:
        raise ValueError("distance_to_board must be positive when provided")
    if cfg.mode == "verify":
        if cfg.verify_fit not in {"grid", "sweep"}:
            raise ValueError("verify.fit must be 'grid' or 'sweep'")
        if cfg.verify_poly_degree < 1:
            raise ValueError("verify.poly_degree must be >= 1")
        if cfg.verify_fit == "grid":
            if cfg.verify_grid_csv is None:
                raise ValueError("verify.grid_csv is required when fit=grid")
        else:
            if cfg.verify_sweep_csv_x is None or cfg.verify_sweep_csv_y is None:
                raise ValueError("verify.sweep_csv_x and sweep_csv_y are required when fit=sweep")
        if cfg.no_calib:
            raise ValueError("verify mode requires calibration with homography H (do not use no_calib)")
    if cfg.mode == "verify-sweep":
        if cfg.verify_fit not in {"grid", "sweep"}:
            raise ValueError("verify-sweep.fit must be 'grid' or 'sweep'")
        if cfg.verify_poly_degree < 1:
            raise ValueError("verify-sweep.poly_degree must be >= 1")
        if cfg.verify_fit == "grid":
            if cfg.verify_grid_csv is None:
                raise ValueError("verify-sweep.grid_csv is required when fit=grid")
        else:
            if cfg.verify_sweep_csv_x is None or cfg.verify_sweep_csv_y is None:
                raise ValueError(
                    "verify-sweep.sweep_csv_x and sweep_csv_y are required when fit=sweep"
                )
        if cfg.no_calib:
            raise ValueError(
                "verify-sweep requires calibration with homography H (do not use no_calib)"
            )
    if cfg.mode == "verify-grid":
        if cfg.verify_fit not in {"grid", "sweep"}:
            raise ValueError("verify-grid.fit must be 'grid' or 'sweep'")
        if cfg.verify_poly_degree < 1:
            raise ValueError("verify-grid.poly_degree must be >= 1")
        if cfg.verify_fit == "grid":
            if cfg.verify_grid_csv is None:
                raise ValueError("verify-grid.grid_csv is required when fit=grid")
        else:
            if cfg.verify_sweep_csv_x is None or cfg.verify_sweep_csv_y is None:
                raise ValueError(
                    "verify-grid.sweep_csv_x and sweep_csv_y are required when fit=sweep"
                )
        if cfg.no_calib:
            raise ValueError(
                "verify-grid requires calibration with homography H (do not use no_calib)"
            )


def _poly_terms_2d_count(degree: int) -> int:
    return (degree + 1) * (degree + 2) // 2


def _design_matrix_2d(vx: np.ndarray, vy: np.ndarray, degree: int) -> np.ndarray:
    vx = np.asarray(vx, dtype=np.float64).ravel()
    vy = np.asarray(vy, dtype=np.float64).ravel()
    cols: list[np.ndarray] = []
    for sum_deg in range(degree + 1):
        for i in range(sum_deg + 1):
            j = sum_deg - i
            cols.append((vx**i) * (vy**j))
    return np.column_stack(cols) if cols else np.zeros((len(vx), 0))


def _fit_multivariate_poly_mm(
    vx: np.ndarray, vy: np.ndarray, z_mm: np.ndarray, degree: int
) -> np.ndarray:
    a = _design_matrix_2d(vx, vy, degree)
    coef, _, rank, _ = np.linalg.lstsq(a, np.asarray(z_mm, dtype=np.float64).ravel(), rcond=None)
    if rank < a.shape[1]:
        print(
            "Warning: polynomial design matrix is rank-deficient; fit may be unstable.",
            file=sys.stderr,
        )
    return coef


def _predict_multivariate_mm(vx: float, vy: float, coef: np.ndarray, degree: int) -> float:
    row = _design_matrix_2d(np.array([vx]), np.array([vy]), degree)
    return float(row @ coef)


def _load_mapping_xy_mm_rows(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    path = _resolve_script_path(path)
    if not path.is_file():
        raise FileNotFoundError(f"CSV not found: {path}")
    vx_list: list[float] = []
    vy_list: list[float] = []
    xm_list: list[float] = []
    ym_list: list[float] = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            header_raw = next(reader)
        except StopIteration as exc:
            raise ValueError(f"Empty CSV: {path}") from exc
        header = {_norm_csv_header(h): i for i, h in enumerate(header_raw)}
        req = ("vdiffx", "vdiffy", "x_mm", "y_mm")
        if not all(k in header for k in req):
            raise ValueError(f"CSV {path} must have columns: {', '.join(req)}")
        ix, iy, imx, imy = (header[k] for k in req)
        for row in reader:
            if len(row) <= max(ix, iy, imx, imy):
                continue
            try:
                vxa = float(row[ix])
                vya = float(row[iy])
                xma = float(row[imx])
                yma = float(row[imy])
            except ValueError:
                continue
            if not all(math.isfinite(v) for v in (vxa, vya, xma, yma)):
                continue
            vx_list.append(vxa)
            vy_list.append(vya)
            xm_list.append(xma)
            ym_list.append(yma)
    if len(vx_list) < 4:
        raise ValueError(f"Too few valid data rows in {path}")
    return (
        np.array(vx_list, dtype=np.float64),
        np.array(vy_list, dtype=np.float64),
        np.array(xm_list, dtype=np.float64),
        np.array(ym_list, dtype=np.float64),
    )


def _fit_verify_grid_models(path: Path, degree: int) -> tuple[np.ndarray, np.ndarray]:
    vx, vy, xm, ym = _load_mapping_xy_mm_rows(path)
    n_terms = _poly_terms_2d_count(degree)
    if len(vx) < n_terms + 1:
        raise ValueError(
            f"Grid CSV has {len(vx)} points; need at least {n_terms + 1} for degree-{degree} "
            "multivariate fit."
        )
    bx = _fit_multivariate_poly_mm(vx, vy, xm, degree)
    by = _fit_multivariate_poly_mm(vx, vy, ym, degree)
    return bx, by


def _fit_verify_sweep_models(path_x: Path, path_y: Path, degree: int) -> tuple[np.ndarray, np.ndarray]:
    vx_a, _vy_a, xm_a, _ym_a = _load_mapping_xy_mm_rows(path_x)
    _vx_b, vy_b, _xm_b, ym_b = _load_mapping_xy_mm_rows(path_y)
    if len(vx_a) < degree + 1:
        raise ValueError(f"X sweep CSV has too few rows for degree {degree}")
    if len(vy_b) < degree + 1:
        raise ValueError(f"Y sweep CSV has too few rows for degree {degree}")
    coef_x = np.polyfit(vx_a, xm_a, degree)
    coef_y = np.polyfit(vy_b, ym_b, degree)
    return coef_x, coef_y


def _make_verify_expected_predictor(cfg: RunConfig) -> Callable[[float, float], tuple[float, float]]:
    """
    Same polynomial features and evaluation as verify mode: (vdiffx, vdiffy) -> expected (x_mm, y_mm).
    Prints a short fit summary to stdout.
    """
    deg = cfg.verify_poly_degree
    if cfg.verify_fit == "grid":
        if cfg.verify_grid_csv is None:
            raise ValueError("grid_csv required for grid fit")
        p = _resolve_script_path(cfg.verify_grid_csv)
        grid_bx, grid_by = _fit_verify_grid_models(cfg.verify_grid_csv, deg)
        print(
            f"[VERIFY] Grid multivariate fit (x_mm,y_mm ~ f(vdiffx,vdiffy)), degree={deg}, from {p}"
        )

        def pred(vx: float, vy: float) -> tuple[float, float]:
            return (
                _predict_multivariate_mm(vx, vy, grid_bx, deg),
                _predict_multivariate_mm(vx, vy, grid_by, deg),
            )

        return pred

    if cfg.verify_sweep_csv_x is None or cfg.verify_sweep_csv_y is None:
        raise ValueError("sweep_csv_x and sweep_csv_y required for sweep fit")
    px = _resolve_script_path(cfg.verify_sweep_csv_x)
    py = _resolve_script_path(cfg.verify_sweep_csv_y)
    sweep_cx, sweep_cy = _fit_verify_sweep_models(cfg.verify_sweep_csv_x, cfg.verify_sweep_csv_y, deg)
    print(
        f"[VERIFY] Sweep fits: x_mm ~ poly(vdiffx), y_mm ~ poly(vdiffy), degree={deg}\n"
        f"         x data: {px}\n"
        f"         y data: {py}"
    )

    def pred(vx: float, vy: float) -> tuple[float, float]:
        return float(np.polyval(sweep_cx, vx)), float(np.polyval(sweep_cy, vy))

    return pred


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


def _start_vdiff_for_mode(cfg: RunConfig) -> tuple[float, float]:
    """Compute the first commanded (vdiffx, vdiffy) point for each measurement mode."""
    if cfg.mode == "man":
        return float(cfg.manual_x), float(cfg.manual_y)
    if cfg.mode in {"sweep", "verify-sweep"}:
        if cfg.axis == "x":
            return float(cfg.start_vdiff), 0.0
        if cfg.axis == "y":
            return 0.0, float(cfg.start_vdiff)
        raise ValueError("sweep.axis must be x or y")
    if cfg.mode in {"grid", "verify-grid"}:
        return float(cfg.grid_start_x), float(cfg.grid_start_y)
    raise ValueError(f"Unsupported measurement mode for startup positioning: {cfg.mode}")


def _position_to_start(cfg: RunConfig, fsm: FSM) -> None:
    """
    Move the mirror to the first mapping coordinate and allow settling before capture.
    """
    vx0, vy0 = _start_vdiff_for_mode(cfg)
    print(f"[STARTUP] moving to start vdiff=({vx0}, {vy0})")
    fsm.set_vdiff(vx0, vy0)
    if cfg.settling_time > 0:
        time.sleep(cfg.settling_time)


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


def _rows_with_verify_columns(
    rows: list[list[Any]],
    header: list[str],
    expected_mm: Callable[[float, float], tuple[float, float]],
) -> tuple[list[list[Any]], list[str]]:
    """Append expected_x, expected_y, difference_x, difference_y (mm; diff = actual − expected)."""
    hdr = list(header)
    need = ("vdiffx", "vdiffy", "x_mm", "y_mm")
    if not all(k in hdr for k in need):
        raise ValueError(f"Rows must include columns {need} (homography calibration).")
    ix_vx = hdr.index("vdiffx")
    ix_vy = hdr.index("vdiffy")
    ix_xm = hdr.index("x_mm")
    ix_ym = hdr.index("y_mm")
    out_header = [*hdr, "expected_x", "expected_y", "difference_x", "difference_y"]
    out_rows: list[list[Any]] = []
    for row in rows:
        vx = float(row[ix_vx])
        vy = float(row[ix_vy])
        ax = float(row[ix_xm])
        ay = float(row[ix_ym])
        ex, ey = expected_mm(vx, vy)
        out_rows.append([*row, ex, ey, ax - ex, ay - ey])
    return out_rows, out_header


def _run_man_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    rows: list[list[Any]] = []
    print("[MAN] Type 'q' to stop and write CSV.")
    print(f"[MAN] start position vdiff=({cfg.manual_x}, {cfg.manual_y})")
    try:
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


def _run_verify_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    """Manual vdiff positions vs polynomial expectations from prior mapping CSV(s)."""
    if calib is None or calib.H is None:
        print("verify mode requires camera calibration with homography H (x_mm / y_mm).", file=sys.stderr)
        return 1

    expected_mm = _make_verify_expected_predictor(cfg)

    out_header = [
        "vdiffx",
        "vdiffy",
        "expected_x_mm",
        "expected_y_mm",
        "actual_x_mm",
        "actual_y_mm",
        "err_x_mm",
        "err_y_mm",
        "cx_ud_px",
        "cy_ud_px",
    ]
    rows_out: list[list[Any]] = []

    print("[VERIFY] Enter vdiff pairs like manual mode: '<vdiffx> <vdiffy>' or q to quit.")
    print("[VERIFY] Expected vs actual are board-plane mm from homography (same as mapping CSV).")

    try:
        while True:
            usr = input("vdiffx vdiffy (or q): ").strip()
            if usr.lower() in {"q", "quit", "exit"}:
                break
            parts = usr.split()
            if len(parts) != 2:
                print("Expected two numbers: vdiffx vdiffy", file=sys.stderr)
                continue
            try:
                vx_cmd = float(parts[0])
                vy_cmd = float(parts[1])
                fsm.set_vdiff(vx_cmd, vy_cmd)
            except ValueError:
                print("Bad numbers.", file=sys.stderr)
                continue
            except UnsafeVoltageRequest as exc:
                print(f"Unsafe: {exc}", file=sys.stderr)
                continue

            if cfg.settling_time > 0:
                time.sleep(cfg.settling_time)

            centroid = mapper.capture_centroid_averages(cam, cfg.num_frames, cfg.roi, calib)
            vx_act, vy_act = fsm.get_voltages()
            exp_x, exp_y = expected_mm(float(vx_act), float(vy_act))

            if len(centroid) < 4 or any(not math.isfinite(centroid[i]) for i in range(4)):
                print("[VERIFY] No valid centroid this sample; skipping CSV row.", file=sys.stderr)
                continue

            cx_ud, cy_ud, ax_mm, ay_mm = (float(centroid[0]), float(centroid[1]), float(centroid[2]), float(centroid[3]))
            err_x = ax_mm - exp_x
            err_y = ay_mm - exp_y
            print(
                f"[VERIFY] vdiff=({vx_act:.4f},{vy_act:.4f}) mm exp=({exp_x:.6f},{exp_y:.6f}) "
                f"act=({ax_mm:.6f},{ay_mm:.6f}) err=({err_x:.6f},{err_y:.6f})"
            )
            rows_out.append(
                [
                    vx_act,
                    vy_act,
                    exp_x,
                    exp_y,
                    ax_mm,
                    ay_mm,
                    err_x,
                    err_y,
                    cx_ud,
                    cy_ud,
                ]
            )
    except KeyboardInterrupt:
        print("\nInterrupted; writing verify CSV if any rows.")

    outfile = _resolve_script_path(cfg.outfile)
    mapper.write_csv(outfile, rows_out, out_header, mode="w")
    print(f"Wrote {outfile} ({len(rows_out)} rows)")
    return 0


def _run_verify_sweep_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    """Automatic sweep with polynomial expectation columns (same sweep CSV shape + verify extras)."""
    if calib is None or calib.H is None:
        print(
            "verify-sweep requires camera calibration with homography H (x_mm / y_mm).",
            file=sys.stderr,
        )
        return 1

    expected_mm = _make_verify_expected_predictor(cfg)

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
    header = list(mapper.csv_header(calib))
    rows, header = _augment_rows_with_angles(rows, header, cfg.distance_to_board)
    try:
        out_rows, out_header = _rows_with_verify_columns(rows, header, expected_mm)
    except ValueError as exc:
        print(f"verify-sweep: {exc}", file=sys.stderr)
        return 1

    outfile = _resolve_script_path(cfg.outfile)
    mapper.write_csv(outfile, out_rows, out_header, mode="w")
    print(f"[VERIFY-SWEEP] Wrote {outfile} ({len(out_rows)} rows)")
    return 0


def _run_verify_grid_mode(cfg: RunConfig, mapper: MappingService, fsm: FSM, cam: Any, calib) -> int:
    """Automatic grid traversal with polynomial expectation columns (same grid CSV shape + verify extras)."""
    if calib is None or calib.H is None:
        print(
            "verify-grid requires camera calibration with homography H (x_mm / y_mm).",
            file=sys.stderr,
        )
        return 1

    expected_mm = _make_verify_expected_predictor(cfg)

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

    header = list(mapper.csv_header(calib))
    rows, header = _augment_rows_with_angles(rows, header, cfg.distance_to_board)
    try:
        out_rows, out_header = _rows_with_verify_columns(rows, header, expected_mm)
    except ValueError as exc:
        print(f"verify-grid: {exc}", file=sys.stderr)
        return 1

    outfile = _resolve_script_path(cfg.outfile)
    mapper.write_csv(outfile, out_rows, out_header, mode="w")
    print(f"[VERIFY-GRID] Wrote {outfile} ({len(out_rows)} rows)")
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
        if cfg.mode != "verify":
            _position_to_start(cfg, fsm)
        if cfg.mode == "man":
            return _run_man_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "sweep":
            return _run_sweep_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "grid":
            return _run_grid_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "verify":
            return _run_verify_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "verify-sweep":
            return _run_verify_sweep_mode(cfg, mapper, fsm, cam, calib)
        if cfg.mode == "verify-grid":
            return _run_verify_grid_mode(cfg, mapper, fsm, cam, calib)
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
