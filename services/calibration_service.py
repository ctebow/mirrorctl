from __future__ import annotations

from pathlib import Path

from src import centroiding
from src.exceptions import CalibrationError


class CalibrationService:
    """Load and validate ``CameraCalibration`` without camera I/O."""

    @staticmethod
    def load(path: Path) -> centroiding.CameraCalibration:
        try:
            return centroiding.CameraCalibration.load(path)
        except FileNotFoundError as exc:
            raise CalibrationError(f"Calibration file not found: {path}") from exc
        except ValueError as exc:
            raise CalibrationError(str(exc)) from exc
