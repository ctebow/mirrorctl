"""
ChArUco lens calibration and homography helpers (OpenCV).

Capture calibration images with get_calib_photos.py (Picamera2), then run this
script to produce camera_params.npz next to this file.
"""
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

_CONFIG_DIR = Path(__file__).resolve().parent
CALIB_IMAGES_DIR = _CONFIG_DIR / "calib_images"
OUTPUT_NPZ = _CONFIG_DIR / "camera_params.npz"

SQUARES_X = 5              # Number of squares in X direction
SQUARES_Y = 7              # Number of squares in Y direction
SQUARE_LENGTH = 0.03       # The size of a square side (e.g. 0.03 for 30mm)
MARKER_LENGTH = 0.02       # The size of the ArUco marker side
DICT_TYPE = aruco.DICT_4X4_50

# Minimum poses for a stable solve (adjust if needed)
MIN_CALIB_IMAGES = 3
MAX_RMS_PIXELS = 5.0       # Reprojection RMS; raise if worse (add more varied images)

dictionary = aruco.getPredefinedDictionary(DICT_TYPE)
board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
detector_params = aruco.DetectorParameters()
detector = aruco.CharucoDetector(board)


def calibrate_lens(image_files):
    """
    Stage A: Lens undistortion from ChArUco views.

    Returns:
        tuple: (rms, mtx, dist)  RMS reprojection error (pixels), camera matrix, distortion coeffs.

    Raises:
        ValueError: too few valid images or unreadable files.
        RuntimeError: OpenCV calibration failed or RMS too high.
    """
    all_charuco_corners = []
    all_charuco_ids = []
    img_size = None

    paths = [Path(p) for p in image_files]
    if len(paths) < MIN_CALIB_IMAGES:
        raise ValueError(
            f"Need at least {MIN_CALIB_IMAGES} images; got {len(paths)}"
        )

    for fname in paths:
        img = cv2.imread(str(fname))
        if img is None:
            raise ValueError(f"Could not read image: {fname}")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)

        if charuco_ids is not None and len(charuco_ids) > 4:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

    if len(all_charuco_corners) < MIN_CALIB_IMAGES:
        raise ValueError(
            f"Need ChArUco detected in at least {MIN_CALIB_IMAGES} images; "
            f"only {len(all_charuco_corners)} succeeded (check board params / lighting)."
        )

    rms, mtx, dist, _rvecs, _tvecs = aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, img_size, None, None
    )

    if mtx is None or dist is None:
        raise RuntimeError("calibrateCameraCharuco failed (None matrix or distortion)")

    if not np.isfinite(rms):
        raise RuntimeError(f"Invalid calibration RMS: {rms}")

    if rms > MAX_RMS_PIXELS:
        raise RuntimeError(
            f"Calibration RMS reprojection error too high: {rms:.3f} px "
            f"(limit {MAX_RMS_PIXELS}); use more images and varied poses."
        )

    return rms, mtx, dist


def get_homography_matrix(frame, mtx, dist):
    """
    Stage B: Homography (keystone) — maps undistorted pixels to board plane (mm).
    """
    undistorted = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    corners, ids, _, _ = detector.detectBoard(gray)

    if ids is not None and len(ids) >= 4:
        obj_points, img_points = board.matchImagePoints(corners, ids)

        src_pts = img_points.reshape(-1, 2)
        dst_pts = obj_points[:, :2].reshape(-1, 2)

        H, _ = cv2.findHomography(src_pts, dst_pts)
        return H, undistorted

    return None, undistorted


def get_laser_position_mm(frame, mtx, dist, H):
    """
    Stage C: Brightest point in undistorted image → mm via homography H.
    """
    undistorted = cv2.undistort(frame, mtx, dist)

    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    _, _max_val, _, max_loc = cv2.minMaxLoc(gray)

    pixel_point = np.array([[list(max_loc)]], dtype="float32")
    world_point = cv2.perspectiveTransform(pixel_point, H)

    return world_point[0][0]


if __name__ == "__main__":
    images = sorted(CALIB_IMAGES_DIR.glob("*.jpg"))
    if not images:
        raise SystemExit(f"No JPG files in {CALIB_IMAGES_DIR}")

    rms, mtx, dist = calibrate_lens(images)
    np.savez(OUTPUT_NPZ, mtx=mtx, dist=dist, rms=rms)
    print(f"Lens calibration saved to {OUTPUT_NPZ} (RMS: {rms:.4f} px)")
