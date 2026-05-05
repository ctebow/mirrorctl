from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Union

import cv2
import numpy as np

from src.constants import (
    LASER_CENTROID_INTENSITY_THRESHOLD,
    LASER_CENTROID_THRESHOLD_ENABLED,
)


def find_laser_centroid(
    gray,
    roi_size=50,
    threshold: Optional[float] = None,
) -> Optional[Tuple[float, float]]:
    """
    Return global coords of laser centroid on the given grayscale image
    (raw or already undistorted).
    """
    h, w = gray.shape

    _, _, _, max_loc = cv2.minMaxLoc(gray)
    x0, y0 = max_loc

    half = roi_size // 2
    x_min = max(x0 - half, 0)
    x_max = min(x0 + half, w)
    y_min = max(y0 - half, 0)
    y_max = min(y0 + half, h)

    roi = gray[y_min:y_max, x_min:x_max]

    ys, xs = np.indices(roi.shape)
    weights = roi.astype(np.float32)
    if threshold is None and LASER_CENTROID_THRESHOLD_ENABLED:
        threshold = float(LASER_CENTROID_INTENSITY_THRESHOLD)
    if threshold is not None:
        weights[weights < float(threshold)] = 0.0

    total = np.sum(weights)
    if total == 0:
        return None

    cx_local = np.sum(xs * weights) / total
    cy_local = np.sum(ys * weights) / total

    cx_global = cx_local + x_min
    cy_global = cy_local + y_min

    return cx_global, cy_global


@dataclass
class CameraCalibration:
    """Lens intrinsics from ChArUco calibration; optional homography to board plane (mm)."""

    mtx: np.ndarray
    dist: np.ndarray
    H: Optional[np.ndarray] = None

    @classmethod
    def load(cls, path: Union[str, Path]) -> "CameraCalibration":
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"Camera calibration not found: {path}")
        data = np.load(path, allow_pickle=False)
        if "mtx" not in data or "dist" not in data:
            raise ValueError(f"Invalid calibration file (need mtx, dist): {path}")
        mtx = data["mtx"]
        dist = data["dist"]
        H = data["H"] if "H" in data.files else None
        return cls(mtx=mtx, dist=dist, H=H)

    def undistort_gray(self, gray: np.ndarray) -> np.ndarray:
        return cv2.undistort(gray, self.mtx, self.dist)

    def find_corrected_rectified_centroid(self, gray: np.ndarray, roi_size: int = 50):
        """
        Undistort frame, find centroid, then optionally map to board mm via H.

        Returns:
            None if no laser signal.
            (cx_px, cy_px) in undistorted pixel space if H is None.
            (x_mm, y_mm) on the ChArUco board plane if H is set.
        """
        rect = self.undistort_gray(gray)
        pt = find_laser_centroid(rect, roi_size)
        if pt is None:
            return None
        if self.H is None:
            return float(pt[0]), float(pt[1])

        pix = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
        world = cv2.perspectiveTransform(pix, self.H)
        return float(world[0, 0, 0]), float(world[0, 0, 1])


def grayscale_to_outfile(gray, outfile):
    cv2.imwrite(outfile, gray)
