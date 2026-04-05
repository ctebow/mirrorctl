"""
ChArUco calibration capture using Picamera2 (same pipeline as src/picam + mapping).

Saves BGR JPEGs for config/calibrate_picam.py (cv2.imread + BGR2GRAY).
"""
import sys
from pathlib import Path

import cv2

# Repo root (parent of config/)
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src import picam

# Match voltage_mapping_main defaults / picam.DEFAULT_FRAME_SIZE
SAVE_DIR = Path(__file__).resolve().parent / "calib_images"
FILE_PREFIX = "charuco_shot"
# Same size tuple as init_camera — change together with mapping CLI --resolution
FRAME_SIZE = picam.DEFAULT_FRAME_SIZE

SAVE_DIR.mkdir(parents=True, exist_ok=True)

print("--- Calibration Capture Utility (Picamera2) ---")
print(f"Resolution: {FRAME_SIZE} (must match mapping run)")
print("1. Aim camera at the ChArUco board.")
print("2. Press SPACE to save a frame.")
print("3. Press ESC to exit.")

picam2 = picam.init_camera(FRAME_SIZE)
try:
    count = 0
    while True:
        gray = picam.get_gray_frame(picam2)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        display_frame = bgr.copy()
        cv2.putText(
            display_frame,
            f"Saved: {count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        cv2.imshow("Calibration Capture", display_frame)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break
        if key == 32:  # SPACE
            file_path = SAVE_DIR / f"{FILE_PREFIX}_{count:02d}.jpg"
            cv2.imwrite(str(file_path), bgr)
            print(f"Saved: {file_path}")
            count += 1
finally:
    picam.close_camera(picam2)
    cv2.destroyAllWindows()
