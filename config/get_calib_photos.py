"""
ChArUco calibration capture using Picamera2 (same pipeline as src/picam + mapping).

Headless/SSH-friendly: no GUI. Type commands on stdin (see below).

Saves BGR JPEGs for config/calibrate_picam.py (cv2.imread + BGR2GRAY).

Usage (on Pi):
  python3 config/get_calib_photos.py
  python3 config/get_calib_photos.py --width 640

Commands (stdin, one per line):
  s, save   — save current frame to config/calib_images/
  q, quit   — exit

Tip: If lines seem buffered over SSH, run:  python3 -u config/get_calib_photos.py
"""
import argparse
import sys
from pathlib import Path

import cv2

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src import picam

SAVE_DIR = Path(__file__).resolve().parent / "calib_images"
FILE_PREFIX = "charuco_shot"


def main():
    parser = argparse.ArgumentParser(description="CLI calibration capture (Picamera2, no preview window).")
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help=f"Frame width; height 480. Default {picam.DEFAULT_FRAME_SIZE[0]}.",
    )
    args = parser.parse_args()
    frame_size = picam.normalize_resolution(args.width) if args.width is not None else picam.DEFAULT_FRAME_SIZE

    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    print("--- Calibration Capture (CLI, Picamera2) ---")
    print(f"Resolution: {frame_size} (match voltage_mapping_main --resolution)")
    print(f"Saving under: {SAVE_DIR}")
    print("Commands:  s / save  → save frame    q / quit  → exit")
    print("If input lags over SSH, use: python3 -u config/get_calib_photos.py")
    print()

    count = 0
    picam2 = picam.init_camera(frame_size)
    try:
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd in ("s", "save"):
                gray = picam.get_gray_frame(picam2)
                bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                file_path = SAVE_DIR / f"{FILE_PREFIX}_{count:02d}.jpg"
                cv2.imwrite(str(file_path), bgr)
                print(f"Saved: {file_path}")
                count += 1
            elif cmd in ("",):
                continue
            else:
                print(f"Unknown command: {line!r} — use s or q", file=sys.stderr)
    finally:
        picam.close_camera(picam2)


if __name__ == "__main__":
    main()
