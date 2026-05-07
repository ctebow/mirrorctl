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


def find_laser_centroid_dark_sub_gaussian(
    gray,
    dark_gray,
    roi_size=50,
    threshold: Optional[float] = None,
) -> Optional[Tuple[float, float]]:
    """
    Return global coords of laser centroid using dark subtraction, soft thresholding,
    and a 2D Gaussian fit on an ROI around the brightest pixel.
    """
    if gray.shape != dark_gray.shape:
        raise ValueError("gray and dark_gray must have the same shape")

    h, w = gray.shape
    signal = gray.astype(np.float32) - dark_gray.astype(np.float32)
    signal[signal < 0.0] = 0.0

    if threshold is None and LASER_CENTROID_THRESHOLD_ENABLED:
        threshold = float(LASER_CENTROID_INTENSITY_THRESHOLD)
    if threshold is not None:
        signal -= float(threshold)
        signal[signal < 0.0] = 0.0

    _, max_val, _, max_loc = cv2.minMaxLoc(signal)
    if max_val <= 0.0:
        return None

    x0, y0 = max_loc
    half = roi_size // 2
    x_min = max(x0 - half, 0)
    x_max = min(x0 + half, w)
    y_min = max(y0 - half, 0)
    y_max = min(y0 + half, h)
    roi = signal[y_min:y_max, x_min:x_max]

    ys, xs = np.indices(roi.shape)
    positive = roi > 0.0
    if not np.any(positive):
        return None

    fit_x = xs[positive].astype(np.float32).reshape(-1, 1)
    fit_y = ys[positive].astype(np.float32).reshape(-1, 1)
    fit_z = np.log(roi[positive].astype(np.float32) + 1e-6).reshape(-1, 1)

    if fit_x.shape[0] >= 5:
        A = np.hstack([fit_x * fit_x, fit_y * fit_y, fit_x, fit_y, np.ones_like(fit_x)])
        ok, coeff = cv2.solve(A, fit_z, flags=cv2.DECOMP_SVD)
        if ok:
            a = float(coeff[0, 0])
            b = float(coeff[1, 0])
            c = float(coeff[2, 0])
            d = float(coeff[3, 0])
            if a < 0.0 and b < 0.0:
                cx_local = -c / (2.0 * a)
                cy_local = -d / (2.0 * b)
                if np.isfinite(cx_local) and np.isfinite(cy_local):
                    if 0.0 <= cx_local < roi.shape[1] and 0.0 <= cy_local < roi.shape[0]:
                        return cx_local + x_min, cy_local + y_min

    weights = roi.astype(np.float32)
    total = np.sum(weights)
    if total <= 0.0:
        return None
    cx_local = np.sum(xs * weights) / total
    cy_local = np.sum(ys * weights) / total
    return cx_local + x_min, cy_local + y_min


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
