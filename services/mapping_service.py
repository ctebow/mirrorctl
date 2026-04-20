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
    start_y: float = 0.0
    end_x: float = 0.0
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
    def _validate_vdiff_pair(vx: float, vy: float, *, name: str) -> None:
        if not (VDIFF_MIN_VOLTS <= vx <= VDIFF_MAX_VOLTS):
            raise ValueError(f"{name}.x={vx} not in vdiff range [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]")
        if not (VDIFF_MIN_VOLTS <= vy <= VDIFF_MAX_VOLTS):
            raise ValueError(f"{name}.y={vy} not in vdiff range [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]")

    @staticmethod
    def _axis_points(start: float, end: float, step: float) -> list[float]:
        eps = 1e-9
        if abs(end - start) <= eps:
            return [float(start)]
        if abs(step) <= eps:
            raise ValueError(f"step cannot be 0 when start ({start}) != end ({end})")

        delta = end - start
        if (delta > 0 and step < 0) or (delta < 0 and step > 0):
            raise ValueError(f"step sign {step} does not move start {start} toward end {end}")

        values: list[float] = []
        curr = float(start)
        while True:
            values.append(curr)
            nxt = curr + step
            if (step > 0 and nxt >= end - eps) or (step < 0 and nxt <= end + eps):
                if abs(values[-1] - end) > eps:
                    values.append(float(end))
                break
            curr = nxt
        return values

    @classmethod
    def _build_grid_path(
        cls,
        *,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        step_x: float,
        step_y: float,
    ) -> list[tuple[float, float]]:
        xs = cls._axis_points(start_x, end_x, step_x)
        ys = cls._axis_points(start_y, end_y, step_y)
        n = max(len(xs), len(ys))

        if len(xs) < n:
            xs = xs + [xs[-1]] * (n - len(xs))
        if len(ys) < n:
            ys = ys + [ys[-1]] * (n - len(ys))
        return list(zip(xs, ys))

    def run_grid_sweep(
        self,
        fsm: Any,
        cam: Any,
        params: MappingSweepParams,
        calib: Optional[centroiding.CameraCalibration],
        *,
        progress: Optional[ProgressCallback] = None,
    ) -> List[list]:
        """
        Sweep from (start_x,start_y) -> (end_x,end_y), return to origin, then
        sweep backward from end -> start. Captures the same centroid data as
        auto mode, with a leading direction label per row.
        """
        self._validate_vdiff_pair(params.start_x, params.start_y, name="start")
        self._validate_vdiff_pair(params.end_x, params.end_y, name="end")

        forward = self._build_grid_path(
            start_x=params.start_x,
            start_y=params.start_y,
            end_x=params.end_x,
            end_y=params.end_y,
            step_x=params.step_x,
            step_y=params.step_y,
        )
        reverse = self._build_grid_path(
            start_x=params.end_x,
            start_y=params.end_y,
            end_x=params.start_x,
            end_y=params.start_y,
            step_x=-params.step_x,
            step_y=-params.step_y,
        )

        rows: list[list] = []
        step_idx = 0

        fsm.set_vdiff(params.start_x, params.start_y)
        time.sleep(params.settling_time)

        for vx_cmd, vy_cmd in forward:
            fsm.set_vdiff(vx_cmd, vy_cmd)
            centroid = self.capture_centroid_averages(cam, params.num_frames, params.roi, calib)
            vx, vy = fsm.get_voltages()
            row = ["forward", vx, vy, *centroid]
            rows.append(row)
            if progress:
                progress(step_idx, float(step_idx), row)
            step_idx += 1
            time.sleep(params.settling_time)

        fsm.set_vdiff(0.0, 0.0)
        time.sleep(params.settling_time)
        fsm.set_vdiff(params.end_x, params.end_y)
        time.sleep(params.settling_time)

        for vx_cmd, vy_cmd in reverse:
            fsm.set_vdiff(vx_cmd, vy_cmd)
            centroid = self.capture_centroid_averages(cam, params.num_frames, params.roi, calib)
            vx, vy = fsm.get_voltages()
            row = ["reverse", vx, vy, *centroid]
            rows.append(row)
            if progress:
                progress(step_idx, float(step_idx), row)
            step_idx += 1
            time.sleep(params.settling_time)

        fsm.set_vdiff(0.0, 0.0)
        return rows
