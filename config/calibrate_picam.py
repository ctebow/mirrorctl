"""
ChArUco lens calibration and homography helpers (OpenCV).

Capture calibration images with get_calib_photos.py (Picamera2), then run this
script to produce camera_params.npz next to this file.
"""
import argparse
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

_CONFIG_DIR = Path(__file__).resolve().parent
CALIB_IMAGES_DIR = _CONFIG_DIR / "calib_images"
OUTPUT_NPZ = _CONFIG_DIR / "camera_params.npz"

SQUARES_X = 20        # Number of squares in X direction
SQUARES_Y = 26              # Number of squares in Y direction
SQUARE_LENGTH = 0.0095       # The size of a square side (e.g. 0.03 for 30mm)
MARKER_LENGTH = 0.007       # The size of the ArUco marker side
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


def update_homography_only(ref_image: Path) -> None:
    """Load existing camera_params.npz and replace/add H from a single board image."""
    if not OUTPUT_NPZ.is_file():
        raise SystemExit(f"Missing {OUTPUT_NPZ}; run lens calibration first.")
    data = np.load(OUTPUT_NPZ, allow_pickle=False)
    mtx = data["mtx"]
    dist = data["dist"]
    rms = float(data["rms"]) if "rms" in data.files else 0.0
    frame = cv2.imread(str(ref_image))
    if frame is None:
        raise SystemExit(f"Could not read image: {ref_image}")
    H, _ = get_homography_matrix(frame, mtx, dist)
    if H is None:
        raise SystemExit("ChArUco board not detected in reference image; cannot compute H.")
    np.savez(OUTPUT_NPZ, mtx=mtx, dist=dist, rms=rms, H=H)
    print(f"Homography saved into {OUTPUT_NPZ}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ChArUco camera calibration for mapping pipeline.")
    parser.add_argument(
        "--homography-ref",
        type=Path,
        default=None,
        help="After lens calibration, compute H from this BGR image (board visible) and save with npz.",
    )
    parser.add_argument(
        "--update-homography",
        type=Path,
        default=None,
        metavar="REF.jpg",
        help="Skip lens calibration; load camera_params.npz and set/replace H from this image only.",
    )
    args = parser.parse_args()

    if args.update_homography is not None:
        update_homography_only(args.update_homography)
        raise SystemExit(0)

    images = sorted(CALIB_IMAGES_DIR.glob("*.jpg"))
    if not images:
        raise SystemExit(f"No JPG files in {CALIB_IMAGES_DIR}")

    rms, mtx, dist = calibrate_lens(images)
    save_kw = dict(mtx=mtx, dist=dist, rms=rms)
    if args.homography_ref is not None:
        ref = cv2.imread(str(args.homography_ref))
        if ref is None:
            raise SystemExit(f"Could not read --homography-ref: {args.homography_ref}")
        H, _ = get_homography_matrix(ref, mtx, dist)
        if H is None:
            raise SystemExit("ChArUco not detected in homography reference; npz saved without H.")
        save_kw["H"] = H
        print("Homography H included in output.")

    np.savez(OUTPUT_NPZ, **save_kw)
    print(f"Lens calibration saved to {OUTPUT_NPZ} (RMS: {rms:.4f} px)")
