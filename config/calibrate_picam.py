"""
ChArUco lens calibration and homography helpers (OpenCV).

Capture calibration images with get_calib_photos.py (Picamera2), then run this
script to produce camera_params.npz next to this file.

Board parameters MUST match the CharUco target you printed (generator PDF / OpenCV
board creation). Use --diagnose if detection fails.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

_CONFIG_DIR = Path(__file__).resolve().parent
CALIB_IMAGES_DIR = _CONFIG_DIR / "calib_images"
OUTPUT_NPZ = _CONFIG_DIR / "camera_params.npz"

# Defaults: common small board — OVERRIDE via CLI to match your printed target exactly.
SQUARES_X = 8
SQUARES_Y = 6
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.023
DICT_TYPE = aruco.DICT_4X4_50

MIN_CALIB_IMAGES = 3
MAX_RMS_PIXELS = 5.0

# #region agent log
_DEBUG_LOG = Path(__file__).resolve().parent.parent / ".cursor" / "debug-499e85.log"


def _agent_log(location: str, message: str, data: dict, hypothesis_id: str) -> None:
    try:
        _DEBUG_LOG.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "sessionId": "499e85",
            "timestamp": int(time.time() * 1000),
            "location": location,
            "message": message,
            "data": data,
            "hypothesisId": hypothesis_id,
        }
        with open(_DEBUG_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload) + "\n")
    except Exception:
        pass


# #endregion


def make_charuco_board(
    squares_x: int,
    squares_y: int,
    square_len: float,
    marker_len: float,
    dict_type: int,
    legacy_pattern: bool = False,
):
    """
    Build Charuco board + CharucoDetector.

    OpenCV 4.7+ uses a **new** CharUco chessboard layout by default. Targets from older
    OpenCV, calib.io, etc. often need ``setLegacyPattern(True)`` or CharUco never
    assembles even when ArUco markers decode (see ``--legacy-charuco``).
    """
    dictionary = aruco.getPredefinedDictionary(dict_type)
    board = aruco.CharucoBoard((squares_x, squares_y), square_len, marker_len, dictionary)
    if legacy_pattern:
        if hasattr(board, "setLegacyPattern"):
            board.setLegacyPattern(True)
        else:
            print(
                "Warning: CharucoBoard has no setLegacyPattern(); use OpenCV 4.7+ or skip --legacy-charuco",
                file=sys.stderr,
            )
    detector = aruco.CharucoDetector(board)
    return board, dictionary, detector


def _detect_aruco_ids(gray: np.ndarray, dictionary):
    """Return (num_markers, sorted_unique_ids list, raw ids ndarray or None)."""
    params = aruco.DetectorParameters()
    if hasattr(aruco, "ArucoDetector"):
        ad = aruco.ArucoDetector(dictionary, params)
        _corners, ids, _rej = ad.detectMarkers(gray)
    else:
        _corners, ids, _rej = aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return 0, [], None
    flat = np.unique(ids.flatten()).astype(int).tolist()
    return int(len(ids)), flat, ids


def diagnose_calibration_images(
    image_paths: list, dictionary, board_desc: dict, charuco_detector=None
) -> None:
    """Print ArUco + CharUco stats on first image to debug detection."""
    if not image_paths:
        print("No images to diagnose.")
        return
    first = image_paths[0]
    img = cv2.imread(str(first))
    if img is None:
        print(f"Diagnose: cannot read {first}")
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    n_mark, uniq_ids, _raw = _detect_aruco_ids(gray, dictionary)
    charuco_n = None
    if charuco_detector is not None:
        _cc, cids, _, _ = charuco_detector.detectBoard(gray)
        charuco_n = None if cids is None else int(len(cids))

    id_preview = uniq_ids[:24] if len(uniq_ids) > 24 else uniq_ids
    suffix = " ..." if len(uniq_ids) > 24 else ""
    print(
        f"Diagnose ({first.name}): ArUco markers detected = {n_mark}\n"
        f"  Unique marker IDs (sorted, sample): {id_preview}{suffix}  (count={len(uniq_ids)})\n"
        f"  CharucoDetector.detectBoard → num_charuco corner ids: {charuco_n}\n"
        f"  If n_mark>0 but charuco is 0/None: try --legacy-charuco, swap --squares-x/--squares-y, "
        f"or confirm dict (DICT_4X4_50 allows ids 0..49 only).\n"
        f"  Square/marker lengths only need consistent **ratio** (30mm & 23mm same as 0.03 & 0.023 m).\n"
        f"  Active model: squares=({board_desc['squares_x']}, {board_desc['squares_y']}), "
        f"dict={board_desc['dict_type']}, legacy={board_desc.get('legacy_charuco', False)}"
    )


def calibrate_lens(image_files, board, detector, dictionary):
    """
    Stage A: Lens undistortion from ChArUco views.
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


def get_homography_matrix(frame, mtx, dist, board, charuco_detector, dictionary=None):
    """
    Stage B: Homography (keystone) — maps undistorted pixels to board plane (mm).
    """
    undistorted = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    # #region agent log
    n_mark_raw = None
    if dictionary is not None:
        n_mark_raw, uniq_raw, _ = _detect_aruco_ids(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dictionary
        )
    n_mark_ud, uniq_ud, _ = (
        _detect_aruco_ids(gray, dictionary) if dictionary is not None else (None, None, None)
    )
    _agent_log(
        "calibrate_picam.py:get_homography_matrix",
        "after undistort, before Charuco detectBoard",
        {
            "undist_gray_shape": list(gray.shape),
            "aruco_on_raw_gray_n": n_mark_raw,
            "aruco_on_undist_gray_n": n_mark_ud,
            "undist_aruco_unique_ids_count": len(uniq_ud) if uniq_ud is not None else None,
        },
        "B",
    )
    # #endregion

    corners, ids, _, _ = charuco_detector.detectBoard(gray)

    # #region agent log
    cid_len = None if ids is None else int(len(ids))
    _agent_log(
        "calibrate_picam.py:get_homography_matrix",
        "after CharucoDetector.detectBoard (undistorted gray)",
        {
            "charuco_ids_len": cid_len,
            "charuco_ids_ge_4": cid_len is not None and cid_len >= 4,
        },
        "A",
    )
    # #endregion

    if ids is not None and len(ids) >= 4:
        obj_points, img_points = board.matchImagePoints(corners, ids)

        src_pts = img_points.reshape(-1, 2)
        dst_pts = obj_points[:, :2].reshape(-1, 2)

        H, _ = cv2.findHomography(src_pts, dst_pts)
        # #region agent log
        _agent_log(
            "calibrate_picam.py:get_homography_matrix",
            "findHomography result",
            {"H_is_none": H is None, "n_src_pts": int(src_pts.shape[0])},
            "E",
        )
        # #endregion
        return H, undistorted

    # #region agent log
    _agent_log(
        "calibrate_picam.py:get_homography_matrix",
        "homography skipped (insufficient Charuco ids)",
        {"charuco_ids_len": cid_len},
        "A",
    )
    # #endregion
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


def update_homography_only(ref_image: Path, board, charuco_detector, dictionary, legacy_charuco: bool) -> None:
    """Load existing camera_params.npz and replace/add H from a single board image."""
    # #region agent log
    _agent_log(
        "calibrate_picam.py:update_homography_only",
        "entry",
        {
            "ref_image": str(ref_image),
            "legacy_charuco_cli": bool(legacy_charuco),
            "npz_has_charuco_legacy": "charuco_legacy" in np.load(OUTPUT_NPZ, allow_pickle=False).files
            if OUTPUT_NPZ.is_file()
            else None,
        },
        "A",
    )
    # #endregion
    if not OUTPUT_NPZ.is_file():
        raise SystemExit(f"Missing {OUTPUT_NPZ}; run lens calibration first.")
    data = np.load(OUTPUT_NPZ, allow_pickle=False)
    mtx = data["mtx"]
    dist = data["dist"]
    rms = float(data["rms"]) if "rms" in data.files else 0.0
    frame = cv2.imread(str(ref_image))
    if frame is None:
        raise SystemExit(f"Could not read image: {ref_image}")
    H, _ = get_homography_matrix(frame, mtx, dist, board, charuco_detector, dictionary)
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
        "--legacy-charuco",
        action="store_true",
        help="Use legacy CharUco chessboard pattern (OpenCV 4.7+). Needed for many PDFs from "
        "older OpenCV / third-party generators when markers decode but CharUco stays empty.",
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
        legacy_pattern=args.legacy_charuco,
    )
    bdesc = _board_desc(
        args.squares_x,
        args.squares_y,
        args.square_length,
        args.marker_length,
        args.dict,
    )
    bdesc["legacy_charuco"] = bool(args.legacy_charuco)

    images = sorted(CALIB_IMAGES_DIR.glob("*.jpg"))

    if args.diagnose:
        diagnose_calibration_images(images, dictionary, bdesc, charuco_detector)
        raise SystemExit(0)

    if args.update_homography is not None:
        # #region agent log
        _agent_log(
            "calibrate_picam.py:__main__",
            "--update-homography board model",
            {
                "squares_x": args.squares_x,
                "squares_y": args.squares_y,
                "dict": args.dict,
                "legacy_charuco": bool(args.legacy_charuco),
            },
            "C",
        )
        # #endregion
        update_homography_only(
            args.update_homography,
            board,
            charuco_detector,
            dictionary,
            args.legacy_charuco,
        )
        raise SystemExit(0)

    if not images:
        raise SystemExit(f"No JPG files in {CALIB_IMAGES_DIR}")

    rms, mtx, dist = calibrate_lens(images, board, charuco_detector, dictionary)
    save_kw = dict(mtx=mtx, dist=dist, rms=rms)
    if args.homography_ref is not None:
        ref = cv2.imread(str(args.homography_ref))
        if ref is None:
            raise SystemExit(f"Could not read --homography-ref: {args.homography_ref}")
        H, _ = get_homography_matrix(ref, mtx, dist, board, charuco_detector, dictionary)
        if H is None:
            raise SystemExit("ChArUco not detected in homography reference; npz saved without H.")
        save_kw["H"] = H
        print("Homography H included in output.")

    np.savez(OUTPUT_NPZ, **save_kw)
    print(f"Lens calibration saved to {OUTPUT_NPZ} (RMS: {rms:.4f} px)")
