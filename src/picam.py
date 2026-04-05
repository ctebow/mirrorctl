"""
Picamera2 helpers shared by voltage_mapping_main and config/get_calib_photos.
Uses the same video configuration so calibration captures match runtime frames.
"""
import time

try:
    from picamera2 import Picamera2
except ImportError as e:
    raise ImportError(
        "picamera2 is required for camera capture on the Raspberry Pi. "
        "Install it with your OS packages (e.g. apt install python3-picamera2)."
    ) from e

# (width, height) — keep in sync with CLI defaults in voltage_mapping_main
DEFAULT_FRAME_SIZE = (1920, 1080)
DEFAULT_FRAME_RATE = 60


def normalize_resolution(resolution):
    """
    Accept int (width, height=480) or (width, height) tuple for init_camera.
    """
    if resolution is None:
        return DEFAULT_FRAME_SIZE
    if isinstance(resolution, int):
        return (resolution, 480)
    if isinstance(resolution, (tuple, list)) and len(resolution) == 2:
        return (int(resolution[0]), int(resolution[1]))
    raise TypeError("resolution must be int width or (width, height) tuple")


def create_yuv420_video_config(picam2, size, frame_rate=DEFAULT_FRAME_RATE):
    """Shared Picamera2 configuration (YUV420 main stream)."""
    return picam2.create_video_configuration(
        main={"format": "YUV420", "size": size},
        controls={"FrameRate": frame_rate},
    )


def init_camera(resolution=None, frame_rate=DEFAULT_FRAME_RATE):
    """
    Start Picamera2 with the same YUV420 pipeline used in production mapping.
    resolution: None → DEFAULT_FRAME_SIZE; int → (int, 480); or (w, h).
    """
    size = normalize_resolution(resolution if resolution is not None else DEFAULT_FRAME_SIZE)
    picam2 = Picamera2()
    config = create_yuv420_video_config(picam2, size, frame_rate)
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    return picam2


def get_gray_frame(picam2):
    frame = picam2.capture_array()
    if frame.ndim == 3:
        gray = frame[:, :, 0]
    else:
        gray = frame
    return gray


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
