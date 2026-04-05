#!/usr/bin/env python3
"""
One-shot (or multi-frame averaged) laser centroid check: capture with Picamera2,
apply the same undistort (+ optional homography) path as voltage mapping,
write metrics to a text file, and save an undistorted grayscale snapshot with
the centroid marked.
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

_DEFAULT_CAL = Path(__file__).resolve().parent / "config" / "camera_params.npz"


def _average_centroids(cam, calib: centroiding.CameraCalibration, num_frames: int, roi: int):
    """
    Return (mean_cx_px, mean_cy_px, mean_out0, mean_out1, last_rect, labels).
    Pixel coords are on the undistorted image. (out0, out1) matches the mapping
    pipeline: undistorted pixels if calib.H is None, else board-plane mm.
    """
    cx_px = []
    cy_px = []
    o0 = []
    o1 = []
    last_rect = None

    for _ in range(num_frames):
        gray = picam.get_gray_frame(cam)
        time.sleep(0.05)
        rect = calib.undistort_gray(gray)
        last_rect = rect
        pt = centroiding.find_laser_centroid(rect, roi)
        if pt is None:
            continue
        cx_px.append(float(pt[0]))
        cy_px.append(float(pt[1]))
        if calib.H is None:
            o0.append(float(pt[0]))
            o1.append(float(pt[1]))
        else:
            pix = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
            world = cv2.perspectiveTransform(pix, calib.H)
            o0.append(float(world[0, 0, 0]))
            o1.append(float(world[0, 0, 1]))

    if not cx_px:
        return None

    labels = ("cx_ud_px", "cy_ud_px") if calib.H is None else ("x_mm", "y_mm")
    return (
        sum(cx_px) / len(cx_px),
        sum(cy_px) / len(cy_px),
        sum(o0) / len(o0),
        sum(o1) / len(o1),
        last_rect,
        labels,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Capture one run, compute laser centroid with camera_params.npz, "
        "write text metrics and an annotated undistorted image.",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=_DEFAULT_CAL,
        help="Path to camera_params.npz (default: config/camera_params.npz).",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=picam.DEFAULT_FRAME_SIZE[0],
        help="Frame width in pixels (height follows picam rules; default: %(default)s).",
    )
    parser.add_argument("--roi", type=int, default=50, help="ROI size around brightest seed (default: %(default)s).")
    parser.add_argument(
        "--num-frames",
        type=int,
        default=5,
        help="Frames to average (default: %(default)s).",
    )
    parser.add_argument(
        "--text-out",
        type=Path,
        default=Path("test_centroid_metrics.txt"),
        help="Text file for centroid and distance-from-center (default: %(default)s).",
    )
    parser.add_argument(
        "--image-out",
        type=Path,
        default=Path("test_centroid_annotated.jpg"),
        help="Undistorted snapshot with centroid circle (default: %(default)s).",
    )
    parser.add_argument(
        "--circle-radius",
        type=int,
        default=12,
        help="Circle radius in pixels on the annotated image (default: %(default)s).",
    )
    args = parser.parse_args(argv)

    try:
        calib = centroiding.CameraCalibration.load(args.calibration)
    except (FileNotFoundError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1

    cam = None
    try:
        cam = picam.init_camera(args.resolution)
    except Exception as exc:
        print(f"Camera failed: {exc}", file=sys.stderr)
        return 1

    try:
        packed = _average_centroids(cam, calib, args.num_frames, args.roi)
    finally:
        picam.close_camera(cam)

    lines: list[str] = []
    if packed is None:
        lines.append("status: no_laser_detected")
        lines.append(f"calibration: {args.calibration.resolve()}")
        args.text_out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print("No centroid found; wrote", args.text_out)
        return 2

    mcxi, mcyi, out0, out1, last_rect, out_labels = packed
    h, w = last_rect.shape
    cx_img = (w - 1) / 2.0
    cy_img = (h - 1) / 2.0
    dist_px = math.hypot(mcxi - cx_img, mcyi - cy_img)

    lines.append("status: ok")
    lines.append(f"calibration: {args.calibration.resolve()}")
    lines.append(f"frames_averaged: {args.num_frames}")
    lines.append(f"undistorted_centroid_px: {mcxi:.4f} {mcyi:.4f}")
    lines.append(
        f"distance_from_image_center_px: {dist_px:.4f}  "
        f"(image_center_px: {cx_img:.4f} {cy_img:.4f})"
    )
    lines.append(f"{out_labels[0]}: {out0:.6f}")
    lines.append(f"{out_labels[1]}: {out1:.6f}")
    if calib.H is not None:
        lines.append(
            f"distance_from_board_origin_mm: {math.hypot(out0, out1):.6f}  "
            f"(sqrt(x_mm^2 + y_mm^2); origin is board frame (0,0))"
        )

    args.text_out.write_text("\n".join(lines) + "\n", encoding="utf-8")

    vis = cv2.cvtColor(last_rect, cv2.COLOR_GRAY2BGR)
    cx_i = int(round(mcxi))
    cy_i = int(round(mcyi))
    cv2.circle(vis, (cx_i, cy_i), args.circle_radius, (0, 255, 0), 2, lineType=cv2.LINE_AA)
    cv2.drawMarker(vis, (cx_i, cy_i), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
    cv2.imwrite(str(args.image_out), vis)

    print("Wrote", args.text_out, "and", args.image_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
