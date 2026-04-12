#!/usr/bin/env python3
"""
Two-position laser distance check: same camera + calibration path as mapping
(undistort, weighted centroid, homography H → board plane mm).

Interactive flow: aim laser at spot 1 → Enter → capture; move to spot 2 → Enter
→ capture; print straight-line distance between centroids in millimetres.

Requires camera_params.npz with homography H (run calibrate_picam with
--homography-ref or --update-homography). Without H, pixel displacement is not
physical mm on the board.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from src import centroiding, picam

_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_CAL = _REPO_ROOT / "config" / "camera_params.npz"


def _average_spot_mm(
    cam,
    calib: centroiding.CameraCalibration,
    num_frames: int,
    roi: int,
):
    """
    Average (x_mm, y_mm) over frames using the same path as voltage_mapping
    get_frames + H. Returns None if no valid centroid in any frame.
    """
    xs: list[float] = []
    ys: list[float] = []
    last_rect = None
    last_cx_ud: float | None = None
    last_cy_ud: float | None = None

    for _ in range(num_frames):
        gray = picam.get_gray_frame(cam)
        time.sleep(0.05)
        rect = calib.undistort_gray(gray)
        last_rect = rect
        pt = centroiding.find_laser_centroid(rect, roi)
        if pt is None:
            continue
        last_cx_ud, last_cy_ud = float(pt[0]), float(pt[1])
        pix = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
        world = cv2.perspectiveTransform(pix, calib.H)
        xs.append(float(world[0, 0, 0]))
        ys.append(float(world[0, 0, 1]))

    if not xs:
        return None

    return (
        sum(xs) / len(xs),
        sum(ys) / len(ys),
        last_rect,
        last_cx_ud,
        last_cy_ud,
    )


def _save_annotated(rect: np.ndarray, cx: float, cy: float, path: Path) -> None:
    vis = cv2.cvtColor(rect, cv2.COLOR_GRAY2BGR)
    ix, iy = int(round(cx)), int(round(cy))
    cv2.drawMarker(
        vis,
        (ix, iy),
        (0, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2,
        line_type=cv2.LINE_AA,
    )
    cv2.circle(vis, (ix, iy), 14, (0, 255, 0), 2, lineType=cv2.LINE_AA)
    cv2.imwrite(str(path), vis)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Two laser positions → Euclidean distance on board plane (mm) via H.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=_DEFAULT_CAL,
        help="camera_params.npz with mtx, dist, and H (default: config/camera_params.npz).",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=picam.DEFAULT_FRAME_SIZE[0],
        help="Frame width (default: %(default)s).",
    )
    parser.add_argument("--roi", type=int, default=50, help="Centroid ROI size (default: %(default)s).")
    parser.add_argument(
        "--num-frames",
        type=int,
        default=5,
        metavar="N",
        help="Frames to average per spot (default: %(default)s).",
    )
    parser.add_argument(
        "--spot1-image",
        type=Path,
        default=None,
        help="If set, save undistorted shot with centroid marked (spot 1).",
    )
    parser.add_argument(
        "--spot2-image",
        type=Path,
        default=None,
        help="If set, save undistorted shot with centroid marked (spot 2).",
    )
    args = parser.parse_args(argv)

    try:
        calib = centroiding.CameraCalibration.load(args.calibration)
    except (FileNotFoundError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1

    if calib.H is None:
        print(
            "This script needs homography H in the npz (board-plane mm).\n"
            "Add H with e.g.:\n"
            "  python3 config/calibrate_picam.py --homography-ref path/to/board.jpg\n"
            "or  python3 config/calibrate_picam.py --update-homography path/to/board.jpg",
            file=sys.stderr,
        )
        return 1

    cam = None
    try:
        cam = picam.init_camera(args.resolution)
    except Exception as exc:
        print(f"Camera failed: {exc}", file=sys.stderr)
        return 1

    try:
        print("Aim laser at SPOT 1 (first position). When stable, press Enter.", flush=True)
        input()

        p1 = _average_spot_mm(cam, calib, args.num_frames, args.roi)
        if p1 is None:
            print("No centroid detected for spot 1.", file=sys.stderr)
            return 2
        x1_mm, y1_mm, rect1, cx1, cy1 = p1
        if args.spot1_image is not None and rect1 is not None and cx1 is not None and cy1 is not None:
            _save_annotated(rect1, cx1, cy1, args.spot1_image)
            print(f"Wrote spot 1 image: {args.spot1_image}", flush=True)

        print("Move laser to SPOT 2 (second position). When stable, press Enter.", flush=True)
        input()

        p2 = _average_spot_mm(cam, calib, args.num_frames, args.roi)
        if p2 is None:
            print("No centroid detected for spot 2.", file=sys.stderr)
            return 2
        x2_mm, y2_mm, rect2, cx2, cy2 = p2
        if args.spot2_image is not None and rect2 is not None and cx2 is not None and cy2 is not None:
            _save_annotated(rect2, cx2, cy2, args.spot2_image)
            print(f"Wrote spot 2 image: {args.spot2_image}", flush=True)

    finally:
        picam.close_camera(cam)

    dx = x2_mm - x1_mm
    dy = y2_mm - y1_mm
    dist_mm = math.hypot(dx, dy)

    print()
    print(f"Spot 1 (board mm): x={x1_mm:.6f}  y={y1_mm:.6f}")
    print(f"Spot 2 (board mm): x={x2_mm:.6f}  y={y2_mm:.6f}")
    print(f"Displacement Δx={dx:.6f} mm  Δy={dy:.6f} mm")
    print(f"Distance traveled: {dist_mm:.6f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
