import cv2
import numpy as np

# Define real-world coordinates of the key points on the keyboard (in mm)
# Example: corners of 3x3 key grid for simplicity
real_world_points = np.array([
    [0, 0, 0],      # Tab
    [86, 20, 0],    # Y
    [136, 20, 0],   # P
    [0, 40, 0],     # Caps Lock
    [66, 60, 0],    # G
    [126, 60, 0],   # L
    [0, 80, 0],     # Shift
    [56, 100, 0],   # V
    [106, 100, 0]   # M
], dtype=np.float32)


# Store object points (same for all images) and image points
object_points = []  # 3D points in the real world
image_points = []   # 2D points in the image plane

images = ["img.png"]

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect key points automatically or annotate them manually
    # Use cv2.goodFeaturesToTrack or a custom detection approach
    # Here, manually input the image coordinates (example for simplicity)
    img_points = np.array([
        [33.41, 89.71], [137.46, 90.22], [209.38, 90.22],  # Row 1 - tab, y, and p keys
        [26.78, 99.40], [118.59, 103.99], [207.34, 101.44],  # Row 2 - caps lock, g, and l keys
        [20.66, 110.62], [100.74, 122.86], [187.45, 119.80]   # Row 3 - shift, v, and m keys
    ], dtype=np.float32)

    object_points.append(real_world_points)
    image_points.append(img_points)

    # Optionally visualize points
    for point in img_points:
        cv2.circle(img, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
    cv2.imshow('Key Points', img)
    cv2.waitKey(10000)

cv2.destroyAllWindows()

# Calibrate the camera
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)

# Print the calibration results
print("Intrinsic Matrix (K):")
print(K)
print("\nDistortion Coefficients (D):")
print(D)

# Save calibration data
np.savez("keyboard_calibration_data.npz", K=K, D=D)
