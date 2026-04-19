from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional

import cv2
import numpy as np

from src import centroiding
from src.constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS


@dataclass
class MappingSweepParams:
    num_frames: int = 5
    settling_time: float = 0.1
    axis: str = "x"
    step_size: float = 1.0
    start: float = 0.0
    end: float = 175.0
    roi: int = 50
    start_x: float = 0.0
    end_x: float = 0.0
    start_y: float = 0.0
    end_y: float = 0.0
    step_x: float = 0.0
    step_y: float = 0.0


ProgressCallback = Callable[[int, float, list], None]


class MappingService:
    """Centroid capture and voltage-axis sweeps (camera + optional calibration)."""

    @staticmethod
    def csv_header(calib: Optional[centroiding.CameraCalibration]) -> list[str]:
        if calib is None:
            return ["vdiffx", "vdiffy", "cx_raw", "cy_raw"]
        if calib.H is not None:
            return ["vdiffx", "vdiffy", "cx_ud_px", "cy_ud_px", "x_mm", "y_mm"]
        return ["vdiffx", "vdiffy", "cx_ud_px", "cy_ud_px"]

    @staticmethod
    def capture_centroid_averages(
        cam: Any,
        num_frames: int,
        roi: int,
        calib: Optional[centroiding.CameraCalibration] = None,
    ) -> list[float]:
        """
        Average centroid over ``num_frames``. Row shape matches ``csv_header(calib)``:
        raw px | undistorted px | undistorted px + board mm (when H present).
        """
        from src import picam

        if calib is not None and calib.H is not None:
            cx, cy, mx, my = [], [], [], []
            for _ in range(num_frames):
                gray = picam.get_gray_frame(cam)
                time.sleep(0.05)
                rect = calib.undistort_gray(gray)
                pt = centroiding.find_laser_centroid(rect, roi)
                if pt is None:
                    continue
                pix = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
                world = cv2.perspectiveTransform(pix, calib.H)
                cx.append(float(pt[0]))
                cy.append(float(pt[1]))
                mx.append(float(world[0, 0, 0]))
                my.append(float(world[0, 0, 1]))
            if not cx:
                return [float("nan"), float("nan"), float("nan"), float("nan")]
            n = len(cx)
            return [sum(cx) / n, sum(cy) / n, sum(mx) / n, sum(my) / n]

        cx, cy = [], []
        for _ in range(num_frames):
            gray = picam.get_gray_frame(cam)
            time.sleep(0.05)
            if calib is None:
                res = centroiding.find_laser_centroid(gray, roi)
            else:
                res = calib.find_corrected_rectified_centroid(gray, roi)
            if res is None:
                continue
            cx.append(res[0])
            cy.append(res[1])

        if not cx:
            return [float("nan"), float("nan")]

        return [sum(cx) / len(cx), sum(cy) / len(cy)]

    @staticmethod
    def write_csv(path: Path, rows: List[list], header: list[str], *, mode: str = "w") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, mode, newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)

    def run_auto_sweep(
        self,
        fsm: Any,
        cam: Any,
        params: MappingSweepParams,
        calib: Optional[centroiding.CameraCalibration],
        *,
        progress: Optional[ProgressCallback] = None,
    ) -> List[list]:
        """Sweep one axis from start to end; returns rows (without header)."""
        if VDIFF_MIN_VOLTS > params.start or params.start > VDIFF_MAX_VOLTS:
            raise ValueError(f"start {params.start} not in vdiff range [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]")
        if VDIFF_MIN_VOLTS > params.end or params.end > VDIFF_MAX_VOLTS:
            raise ValueError(f"end {params.end} not in vdiff range [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]")

        coords: list[list] = []
        curr = params.start
        step = 0

        try:
            while curr <= params.end + 1e-9:
                if params.axis == "x":
                    fsm.set_vdiff(curr, 0.0)
                elif params.axis == "y":
                    fsm.set_vdiff(0.0, curr)
                else:
                    raise ValueError("axis must be 'x' or 'y' for auto sweep")

                centroid = self.capture_centroid_averages(cam, params.num_frames, params.roi, calib)
                vx, vy = fsm.get_voltages()
                row = [vx, vy, *centroid]
                coords.append(row)
                if progress:
                    progress(step, curr, row)
                time.sleep(params.settling_time)
                curr += params.step_size
                step += 1
        finally:
            pass

        return coords

    @staticmethod
    def _inclusive_axis_values(start: float, end: float, step: float) -> list[float]:
        """Build inclusive axis values from start->end using a positive step size."""
        if step <= 0:
            raise ValueError("grid step must be > 0")
        if abs(end - start) < 1e-12:
            return [start]

        vals: list[float] = []
        direction = 1.0 if end >= start else -1.0
        signed_step = direction * step
        curr = start

        if direction > 0:
            while curr <= end + 1e-9:
                vals.append(curr)
                curr += signed_step
        else:
            while curr >= end - 1e-9:
                vals.append(curr)
                curr += signed_step

        if abs(vals[-1] - end) > 1e-6:
            vals.append(end)
        return vals

    def run_grid_sweep(
        self,
        fsm: Any,
        cam: Any,
        params: MappingSweepParams,
        calib: Optional[centroiding.CameraCalibration],
    ) -> List[list]:
        """
        Run a serpentine X/Y sweep:
        - Row 0: x start->end at y0
        - Row 1: x end->start at y1
        - Repeat while stepping one row at a time in y.
        """
        for name, val in (
            ("start_x", params.start_x),
            ("end_x", params.end_x),
            ("start_y", params.start_y),
            ("end_y", params.end_y),
        ):
            if VDIFF_MIN_VOLTS > val or val > VDIFF_MAX_VOLTS:
                raise ValueError(
                    f"{name} {val} not in vdiff range [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]"
                )

        step_x = params.step_x if params.step_x > 0 else params.step_size
        step_y = params.step_y if params.step_y > 0 else params.step_size
        if step_x <= 0 or step_y <= 0:
            raise ValueError("grid step_x/step_y (or step_size) must be > 0")

        x_vals = self._inclusive_axis_values(params.start_x, params.end_x, step_x)
        y_vals = self._inclusive_axis_values(params.start_y, params.end_y, step_y)

        rows: list[list] = []
        for row_idx, y in enumerate(y_vals):
            x_iter = x_vals if row_idx % 2 == 0 else list(reversed(x_vals))
            sweep_dir = "x+" if row_idx % 2 == 0 else "x-"
            for x in x_iter:
                fsm.set_vdiff(x, y)
                centroid = self.capture_centroid_averages(cam, params.num_frames, params.roi, calib)
                vx, vy = fsm.get_voltages()
                rows.append([sweep_dir, vx, vy, *centroid])
                time.sleep(params.settling_time)

        return rows
