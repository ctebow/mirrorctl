import cv2
import os

# --- CONFIGURATION ---
SAVE_DIR = "calib_images"
FILE_PREFIX = "charuco_shot"

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# Initialize camera (0 is usually the default RPi camera)
cap = cv2.VideoCapture(0)

# Set resolution (Match this to your final measurement resolution!)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("--- Calibration Capture Utility ---")
print("1. Aim camera at the ChArUco board.")
print("2. Press SPACE to save a frame.")
print("3. Press ESC to exit.")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Display the count on the screen
    display_frame = frame.copy()
    cv2.putText(display_frame, f"Saved: {count}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow("Calibration Capture", display_frame)

    key = cv2.waitKey(1)
    if key == 27: # ESC
        break
    elif key == 32: # SPACE
        file_path = os.path.join(SAVE_DIR, f"{FILE_PREFIX}_{count:02d}.jpg")
        cv2.imwrite(file_path, frame)
        print(f"Saved: {file_path}")
        count += 1

cap.release()
cv2.destroyAllWindows()