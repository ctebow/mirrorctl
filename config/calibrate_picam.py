"""
ChArUco lens calibration and homography helpers (OpenCV).

Capture calibration images with get_calib_photos.py (Picamera2), then run this
script to produce camera_params.npz next to this file.

Board parameters MUST match the CharUco target you printed (generator PDF / OpenCV
board creation). Use --diagnose if detection fails.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

_CONFIG_DIR = Path(__file__).resolve().parent
CALIB_IMAGES_DIR = _CONFIG_DIR / "calib_images"
OUTPUT_NPZ = _CONFIG_DIR / "camera_params.npz"

# Defaults: common small board — OVERRIDE via CLI to match your printed target exactly.
SQUARES_X = 6
SQUARES_Y = 8
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.023
DICT_TYPE = aruco.DICT_4X4_50

MIN_CALIB_IMAGES = 3
MAX_RMS_PIXELS = 5.0

# #region agent log
_AGENT_LOG_CANDIDATES = (
    Path("/Users/cadentebow/pulseA_FSM/fsm_driver_dev/mirrorcle_driver_src/rasppi_src/.cursor/debug-499e85.log"),
    Path(__file__).resolve().parent.parent / ".cursor" / "debug-499e85.log",
)


def _agent_log(hypothesis_id: str, location: str, message: str, data: dict, run_id: str = "charuco-detect") -> None:
    entry = {
        "sessionId": "499e85",
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    line = json.dumps(entry) + "\n"
    for log_path in _AGENT_LOG_CANDIDATES:
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(line)
            break
        except OSError:
            continue


# #endregion


def make_charuco_board(squares_x: int, squares_y: int, square_len: float, marker_len: float, dict_type: int):
    """Build Charuco board + CharucoDetector for given geometry."""
    dictionary = aruco.getPredefinedDictionary(dict_type)
    board = aruco.CharucoBoard((squares_x, squares_y), square_len, marker_len, dictionary)
    detector = aruco.CharucoDetector(board)
    return board, dictionary, detector


def _count_aruco_markers(gray: np.ndarray, dictionary) -> int:
    """Raw ArUco marker count (Charuco assembly may still fail if grid is wrong)."""
    params = aruco.DetectorParameters()
    if hasattr(aruco, "ArucoDetector"):
        ad = aruco.ArucoDetector(dictionary, params)
        _corners, ids, _rej = ad.detectMarkers(gray)
    else:
        _corners, ids, _rej = aruco.detectMarkers(gray, dictionary, parameters=params)
    return 0 if ids is None else int(len(ids))


def diagnose_calibration_images(image_paths: list, dictionary, board_desc: dict) -> None:
    """Print and log ArUco counts on first image to separate dict vs Charuco-grid issues."""
    if not image_paths:
        print("No images to diagnose.")
        return
    first = image_paths[0]
    img = cv2.imread(str(first))
    if img is None:
        print(f"Diagnose: cannot read {first}")
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    n_mark = _count_aruco_markers(gray, dictionary)
    print(
        f"Diagnose ({first.name}): ArUco markers detected = {n_mark}\n"
        f"  If 0: wrong DICT (--dict), blur, glare, or board not in frame.\n"
        f"  If >0 but calibration still finds 0 CharUco corners: wrong --squares-x/--squares-y "
        f"or --square-length / --marker-length vs printed board.\n"
        f"  Active model: squares=({board_desc['squares_x']}, {board_desc['squares_y']}), "
        f"dict={board_desc['dict_type']}"
    )
    # #region agent log
    _agent_log(
        "H1",
        "calibrate_picam.diagnose",
        "aruco_marker_count_first_image",
        {"file": first.name, "num_aruco_markers": n_mark, **board_desc},
        run_id="post-fix",
    )
    # #endregion


def calibrate_lens(image_files, board, detector, dictionary, board_log: dict):
    """
    Stage A: Lens undistortion from ChArUco views.
    board_log: dict passed to agent logging (squares, lengths, dict).
    """
    all_charuco_corners = []
    all_charuco_ids = []
    img_size = None

    paths = [Path(p) for p in image_files]
    # #region agent log
    _agent_log(
        "H1_H2",
        "calibrate_picam.calibrate_lens:start",
        "board_config_and_inputs",
        {
            "opencv_version": cv2.__version__,
            **board_log,
            "calib_images_count": len(paths),
            "first_three_names": [paths[i].name for i in range(min(3, len(paths)))],
            "min_ids_required_gt": 4,
        },
        run_id="post-fix",
    )
    # #endregion
    if len(paths) < MIN_CALIB_IMAGES:
        raise ValueError(
            f"Need at least {MIN_CALIB_IMAGES} images; got {len(paths)}"
        )

    for fname in paths:
        img = cv2.imread(str(fname))
        if img is None:
            # #region agent log
            _agent_log("H5", "calibrate_picam.calibrate_lens:imread", "failed", {"file": str(fname)}, run_id="post-fix")
            # #endregion
            raise ValueError(f"Could not read image: {fname}")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
        n_ids = 0 if charuco_ids is None else int(len(charuco_ids))
        ids_none = charuco_ids is None
        n_aruco = _count_aruco_markers(gray, dictionary)

        # #region agent log
        _agent_log(
            "H3_H4",
            "calibrate_picam.calibrate_lens:detectBoard",
            "per_image",
            {
                "file": fname.name,
                "gray_wh": [int(img_size[0]), int(img_size[1])],
                "charuco_ids_is_none": ids_none,
                "num_charuco_ids": n_ids,
                "num_raw_aruco_markers": n_aruco,
                "accepted": bool(charuco_ids is not None and len(charuco_ids) > 4),
                "corners_shape": None
                if charuco_corners is None
                else list(charuco_corners.shape),
            },
            run_id="post-fix",
        )
        # #endregion

        if charuco_ids is not None and len(charuco_ids) > 4:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

    if len(all_charuco_corners) < MIN_CALIB_IMAGES:
        raise ValueError(
            f"Need ChArUco detected in at least {MIN_CALIB_IMAGES} images; "
            f"only {len(all_charuco_corners)} succeeded (check board params / lighting). "
            f"Run with --diagnose for hints."
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


def get_homography_matrix(frame, mtx, dist, board, charuco_detector):
    """
    Stage B: Homography (keystone) — maps undistorted pixels to board plane (mm).
    """
    undistorted = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    corners, ids, _, _ = charuco_detector.detectBoard(gray)

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


def update_homography_only(ref_image: Path, board, charuco_detector) -> None:
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
    H, _ = get_homography_matrix(frame, mtx, dist, board, charuco_detector)
    if H is None:
        raise SystemExit("ChArUco board not detected in reference image; cannot compute H.")
    np.savez(OUTPUT_NPZ, mtx=mtx, dist=dist, rms=rms, H=H)
    print(f"Homography saved into {OUTPUT_NPZ}")


def _board_desc(sx, sy, sq, mk, dt) -> dict:
    return {
        "squares_x": sx,
        "squares_y": sy,
        "square_length": sq,
        "marker_length": mk,
        "dict_type": int(dt),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ChArUco camera calibration for mapping pipeline.")
    parser.add_argument(
        "--squares-x",
        type=int,
        default=SQUARES_X,
        help="Charuco squares in X (must match printed target).",
    )
    parser.add_argument(
        "--squares-y",
        type=int,
        default=SQUARES_Y,
        help="Charuco squares in Y (must match printed target).",
    )
    parser.add_argument(
        "--square-length",
        type=float,
        default=SQUARE_LENGTH,
        help="Checker square side length (same units as marker length; e.g. meters).",
    )
    parser.add_argument(
        "--marker-length",
        type=float,
        default=MARKER_LENGTH,
        help="ArUco marker side length on the printed board.",
    )
    parser.add_argument(
        "--dict",
        type=int,
        default=int(DICT_TYPE),
        help="OpenCV aruco predefined dictionary id (e.g. cv2.aruco.DICT_4X4_50).",
    )
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="Only analyze first calib image: print raw ArUco marker count and hints.",
    )
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

    board, dictionary, charuco_detector = make_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_length,
        args.marker_length,
        args.dict,
    )
    bdesc = _board_desc(
        args.squares_x,
        args.squares_y,
        args.square_length,
        args.marker_length,
        args.dict,
    )

    images = sorted(CALIB_IMAGES_DIR.glob("*.jpg"))
    # #region agent log
    _agent_log(
        "H2",
        "calibrate_picam.__main__",
        "glob_jpg",
        {"dir": str(CALIB_IMAGES_DIR), "n_jpg": len(images), "sample": [p.name for p in images[:5]]},
        run_id="post-fix",
    )
    # #endregion

    if args.diagnose:
        diagnose_calibration_images(images, dictionary, bdesc)
        raise SystemExit(0)

    if args.update_homography is not None:
        update_homography_only(args.update_homography, board, charuco_detector)
        raise SystemExit(0)

    if not images:
        raise SystemExit(f"No JPG files in {CALIB_IMAGES_DIR}")

    rms, mtx, dist = calibrate_lens(images, board, charuco_detector, dictionary, bdesc)
    save_kw = dict(mtx=mtx, dist=dist, rms=rms)
    if args.homography_ref is not None:
        ref = cv2.imread(str(args.homography_ref))
        if ref is None:
            raise SystemExit(f"Could not read --homography-ref: {args.homography_ref}")
        H, _ = get_homography_matrix(ref, mtx, dist, board, charuco_detector)
        if H is None:
            raise SystemExit("ChArUco not detected in homography reference; npz saved without H.")
        save_kw["H"] = H
        print("Homography H included in output.")

    np.savez(OUTPUT_NPZ, **save_kw)
    print(f"Lens calibration saved to {OUTPUT_NPZ} (RMS: {rms:.4f} px)")
