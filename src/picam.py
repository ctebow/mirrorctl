try:
    from picamera2 import Picamera2
except ImportError:
    ...
import time

def init_camera(resolution=(640, 480)):
    picam2 = Picamera2()
    # Efficient config: YUV420 → fast grayscale (Y channel)
    config = picam2.create_video_configuration(
        main={"format": "YUV420", "size": resolution},
        controls={
            "FrameRate": 60  # adjust as needed
        }
    )
    picam2.configure(config)
    # Start camera
    picam2.start()
    # Let auto-exposure settle (important)
    time.sleep(1)
    return picam2

def get_gray_frame(picam2):
    frame = picam2.capture_array()
    # Y channel = grayscale (fast, no conversion)
    if frame.ndim == 3:
        gray = frame[:, :, 0]
    else:
        gray = frame
    return gray


