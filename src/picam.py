"""
Picamera2 helpers shared by voltage_mapping_main and config/get_calib_photos.

Main stream uses **RGB888** so `capture_array()` is always a normal H×W×3 image.
Grayscale for centroiding and calibration is **RGB → gray** via OpenCV (no YUV420
planar buffer / wrong luma-plane bugs in previews or saved JPEGs).
"""
import time

import cv2

from src.constants import (
    CAMERA_ANALOGUE_GAIN,
    CAMERA_AUTO_EXPOSURE_ENABLED,
    CAMERA_AUTO_WHITE_BALANCE_ENABLED,
    CAMERA_COLOUR_GAINS,
    CAMERA_DEFAULT_HEIGHT,
    CAMERA_DEFAULT_WIDTH,
    CAMERA_EXPOSURE_TIME_US,
    CAMERA_FRAME_RATE,
    CAMERA_MAIN_FORMAT,
)

try:
    from picamera2 import Picamera2
except ImportError as e:
    raise ImportError(
        "picamera2 is required for camera capture on the Raspberry Pi. "
        "If you use a venv: either recreate it with "
        "`python3 -m venv .venv --system-site-packages` after `sudo apt install python3-picamera2`, "
        "or run with the system interpreter (deactivate the venv). "
        "apt installs into system Python, not into an isolated venv."
    ) from e

# (width, height) and stream controls are centralized in src/constants.py
DEFAULT_FRAME_SIZE = (CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT)
DEFAULT_FRAME_RATE = CAMERA_FRAME_RATE


def normalize_resolution(resolution):
    """
    Accept int (width, height=480) or (width, height) tuple for init_camera.
    """
    if resolution is None:
        return DEFAULT_FRAME_SIZE
    if isinstance(resolution, int):
        return (resolution, CAMERA_DEFAULT_HEIGHT)
    if isinstance(resolution, (tuple, list)) and len(resolution) == 2:
        return (int(resolution[0]), int(resolution[1]))
    raise TypeError("resolution must be int width or (width, height) tuple")


def create_video_configuration(picam2, size, frame_rate=DEFAULT_FRAME_RATE):
    """RGB888 main stream — unambiguous array layout for preview and processing."""
    controls = {
        "FrameRate": frame_rate,
        "AeEnable": bool(CAMERA_AUTO_EXPOSURE_ENABLED),
        "AwbEnable": bool(CAMERA_AUTO_WHITE_BALANCE_ENABLED),
    }
    if not CAMERA_AUTO_EXPOSURE_ENABLED:
        controls["ExposureTime"] = int(CAMERA_EXPOSURE_TIME_US)
        controls["AnalogueGain"] = float(CAMERA_ANALOGUE_GAIN)
    if not CAMERA_AUTO_WHITE_BALANCE_ENABLED:
        controls["ColourGains"] = tuple(float(v) for v in CAMERA_COLOUR_GAINS)
    return picam2.create_video_configuration(
        main={"format": CAMERA_MAIN_FORMAT, "size": size},
        controls=controls,
    )


def init_camera(resolution=None, frame_rate=DEFAULT_FRAME_RATE):
    """
    Start Picamera2 with the same RGB pipeline used in production mapping.
    resolution: None → DEFAULT_FRAME_SIZE; int → (int, 480); or (w, h).
    """
    size = normalize_resolution(resolution if resolution is not None else DEFAULT_FRAME_SIZE)
    picam2 = Picamera2()
    config = create_video_configuration(picam2, size, frame_rate)
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    return picam2


def get_gray_frame(picam2):
    """
    Capture main array and convert to single-channel gray (BT.601-style weights).
    Same path for preview stream, calibration saves, and voltage mapping.
    """
    frame = picam2.capture_array()
    if frame.ndim == 2:
        return frame
    if frame.ndim != 3:
        raise ValueError(f"Unexpected capture_array shape {frame.shape!r}")
    c = frame.shape[2]
    if c == 3:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    if c == 4:
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
    raise ValueError(f"Unexpected channel count {c} for shape {frame.shape!r}")


def close_camera(picam2):
    """Stop and close Picamera2; safe to call if already stopped."""
    if picam2 is None:
        return
    try:
        picam2.stop()
    except Exception:
        pass
    try:
        picam2.close()
    except Exception:
        pass
