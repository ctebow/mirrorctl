"""
MADE WITH GEMINI
"""
import glob
import cv2
import cv2.aruco as aruco
import numpy as np

SQUARES_X = 5              # Number of squares in X direction
SQUARES_Y = 7              # Number of squares in Y direction
SQUARE_LENGTH = 0.03       # The size of a square side (e.g., 0.03 for 30mm)
MARKER_LENGTH = 0.02       # The size of the ArUco marker side
DICT_TYPE = aruco.DICT_4X4_50

"""
init steps
"""
dictionary = aruco.getPredefinedDictionary(DICT_TYPE)
board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
detector_params = aruco.DetectorParameters()
detector = aruco.CharucoDetector(board)

def calibrate_lens(image_files):
    """
    Stage A: Lens Undistortion
    Pass a list of image paths containing the ChArUco board at different angles.
    """
    all_charuco_corners = []
    all_charuco_ids = []
    img_size = None

    for fname in image_files:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)

        if charuco_ids is not None and len(charuco_ids) > 4:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

    # Calibrate
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, img_size, None, None
    )
    return mtx, dist

def get_homography_matrix(frame, mtx, dist):
    """
    Stage B: Homography (The "Keystone" Fix)
    Maps undistorted pixels to real-world millimeters.
    """
    # 1. Undistort the frame first
    undistorted = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    # 2. Detect markers on the flat image
    corners, ids, _, _ = detector.detectBoard(gray)

    if ids is not None and len(ids) >= 4:
        # Get the 3D object points and their corresponding 2D image points
        # For a flat screen, we map image points (pixels) to board points (mm)
        obj_points, img_points = board.matchImagePoints(corners, ids)
        
        # We only need X and Y from the board for a 2D Homography
        src_pts = img_points.reshape(-1, 2)
        dst_pts = obj_points[:, :2].reshape(-1, 2) 

        # Calculate Homography (H)
        H, _ = cv2.findHomography(src_pts, dst_pts)
        return H, undistorted
    
    return None, undistorted

def get_laser_position_mm(frame, mtx, dist, H):
    """
    Stage C: Transform Laser Pixel to Real-World MM
    """
    # 1. Undistort
    undistorted = cv2.undistort(frame, mtx, dist)
    
    # 2. Find Laser Spot (Simple Centroid of brightest point)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    _, max_val, _, max_loc = cv2.minMaxLoc(gray)
    
    # Optional: Threshold to find centroid if spot is > 1 pixel
    # ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    # M = cv2.moments(thresh)
    # cx = int(M['m10']/M['m00'])
    # cy = int(M['m01']/M['m00'])

    # 3. Apply Homography to the pixel (max_loc)
    pixel_point = np.array([[list(max_loc)]], dtype='float32')
    world_point = cv2.perspectiveTransform(pixel_point, H)
    
    return world_point[0][0] # Returns (x_mm, y_mm)

# --- EXAMPLE WORKFLOW ---
# 1. Run calibrate_lens() with your 10-20 saved calibration photos
# mtx, dist = calibrate_lens(my_photo_list)

# 2. Get Homography using a live frame of the grid
# H, _ = get_homography_matrix(current_frame, mtx, dist)

# 3. Calculate laser spot in MM
# x_mm, y_mm = get_laser_position_mm(laser_frame, mtx, dist, H)

if __name__ == "__main__":

    # Get all file paths
    images = glob.glob("calib_images/*.jpg")

    # Run calibration
    mtx, dist = calibrate_lens(images)

    # Save these! Don't recalibrate the lens unless you change the physical lens focus
    np.savez("camera_params.npz", mtx=mtx, dist=dist)
    print("Lens calibration parameters saved.")


