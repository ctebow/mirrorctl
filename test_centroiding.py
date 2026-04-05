#!/usr/bin/env python3
"""
One-shot (or multi-frame averaged) laser centroid check: capture with Picamera2,
apply the same undistort (+ optional homography) path as voltage mapping,
write metrics to a text file, and save an undistorted grayscale snapshot with
the pipeline centroid marked and the top distinct bright peaks circled.
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

# BGR, rank 1 .. 5 — distinct on typical gray/laser imagery
_PEAK_COLORS = (
    (0, 255, 0),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
    (0, 165, 255),
)


def _top_bright_spots(
    gray: np.ndarray,
    count: int,
    suppress_radius: int,
) -> list[tuple[int, int, float]]:
    """
    Repeated global max with circular suppression so each spot is a separate peak.
    Each entry is (x, y, max_value_in_masked_region) with x=columns, y=rows.
    """
    work = gray.astype(np.float32, copy=True)
    spots: list[tuple[int, int, float]] = []
    for _ in range(count):
        _, max_val, _, max_loc = cv2.minMaxLoc(work)
        if max_val <= 0:
            break
        x, y = int(max_loc[0]), int(max_loc[1])
        spots.append((x, y, float(max_val)))
        cv2.circle(work, (x, y), suppress_radius, 0.0, thickness=-1)
    return spots


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
        "write text metrics and an annotated undistorted image (centroid + top bright spots).",
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
        help="Undistorted snapshot with annotations (default: %(default)s).",
    )
    parser.add_argument(
        "--circle-radius",
        type=int,
        default=12,
        help="Circle radius in pixels for each bright-spot highlight (default: %(default)s).",
    )
    parser.add_argument(
        "--top-spots",
        type=int,
        default=5,
        help="Number of brightest distinct peaks to circle (default: %(default)s).",
    )
    parser.add_argument(
        "--peak-separation",
        type=int,
        default=None,
        metavar="PX",
        help="Suppression radius between peaks (default: max(roi//2, 2*circle-radius)).",
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

    sep = args.peak_separation
    if sep is None:
        sep = max(args.roi // 2, 2 * args.circle_radius)
    spots = _top_bright_spots(last_rect, max(0, args.top_spots), sep)
    lines.append(f"top_bright_spots: count={len(spots)} suppression_radius_px={sep}")
    lines.append("rank  x_px  y_px  raw_max_in_peak_region")
    for i, (sx, sy, sv) in enumerate(spots, start=1):
        lines.append(f"{i}  {sx}  {sy}  {sv:.2f}")

    args.text_out.write_text("\n".join(lines) + "\n", encoding="utf-8")

    vis = cv2.cvtColor(last_rect, cv2.COLOR_GRAY2BGR)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, (sx, sy, _) in enumerate(spots):
        col = _PEAK_COLORS[i % len(_PEAK_COLORS)]
        cv2.circle(vis, (sx, sy), args.circle_radius, col, 2, lineType=cv2.LINE_AA)
        cv2.putText(
            vis,
            str(i + 1),
            (sx + args.circle_radius + 2, sy + 4),
            font,
            0.5,
            col,
            1,
            lineType=cv2.LINE_AA,
        )
    cx_i = int(round(mcxi))
    cy_i = int(round(mcyi))
    cv2.circle(vis, (cx_i, cy_i), args.circle_radius + 2, (255, 255, 255), 2, lineType=cv2.LINE_AA)
    cv2.drawMarker(
        vis,
        (cx_i, cy_i),
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=16,
        thickness=2,
        line_type=cv2.LINE_AA,
    )
    cv2.imwrite(str(args.image_out), vis)

    print("Wrote", args.text_out, "and", args.image_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
