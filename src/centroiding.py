import cv2
import numpy as np

def find_laser_centroid(gray, roi_size=50) -> tuple:
    """
    Return global coords of laser centroid
    """
    h, w = gray.shape
    
    # get brightest
    _, _, _, max_loc = cv2.minMaxLoc(gray)
    x0, y0 = max_loc

    # ROI bounds (clamp to image)
    half = roi_size // 2
    x_min = max(x0 - half, 0)
    x_max = min(x0 + half, w)
    y_min = max(y0 - half, 0)
    y_max = min(y0 + half, h)

    roi = gray[y_min:y_max, x_min:x_max]

    # Compute centroid (weighted, no threshold)
    ys, xs = np.indices(roi.shape)
    weights = roi.astype(np.float32)

    total = np.sum(weights)
    if total == 0:
        return None  # no signal

    cx_local = np.sum(xs * weights) / total
    cy_local = np.sum(ys * weights) / total

    # Convert to global coordinates
    cx_global = cx_local + x_min
    cy_global = cy_local + y_min

    return cx_global, cy_global

def grayscale_to_outfile(gray, outfile):
    """
    Write grayscale to outfile for visualization
    """

    cv2.imwrite(outfile, gray)