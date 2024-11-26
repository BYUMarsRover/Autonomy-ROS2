import cv2
import numpy as np
import glob

# Specify the dimensions of the chessboard
CHESSBOARD_DIM = (10, 7)
SQUARE_SIZE = 20  # Size of a square in mm

# Prepare object points (3D points in real-world space)
objp = np.zeros((CHESSBOARD_DIM[0] * CHESSBOARD_DIM[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_DIM[0], 0:CHESSBOARD_DIM[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane
corrected_imgpoints = []

# Load calibration images
images = glob.glob("calibration_images/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"Image resolution: {gray.shape}")

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_DIM, None)

    if ret:
        objpoints.append(np.array(objp, dtype=np.float32))
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHESSBOARD_DIM, corners, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)
    else:
        print(f"Chessboard corners not found in image: {fname}")

cv2.destroyAllWindows()

# Perform calibration
K = np.zeros((3, 3))
D = np.zeros((4, 1))
objpoints = [np.array(points, dtype=np.float32) for points in objpoints]
imgpoints = [np.array(points, dtype=np.float32) for points in imgpoints]


corrected_imgpoints = []
for points in imgpoints:
    # If the points array has a shape (70, 1, 2), squeeze it to (70, 2)
    if points.shape[1] == 1:
        points = np.squeeze(points, axis=1)  # Removes the middle dimension
    if points.shape[1] != 2:
        raise ValueError(f"imgpoints should have shape (num_points, 2), but got {points.shape}")

    points = np.array(points, dtype=np.float32)
    corrected_imgpoints.append(points)
imgpoints = [points.squeeze(axis=1) for points in imgpoints]
print(f"objpoints shape: {[o.shape for o in objpoints]}")
print(f"objpoints dtype: {[o.dtype for o in objpoints]}")
print(f"imgpoints shape: {[i.shape for i in imgpoints]}")
print(f"imgpoints dtype: {[i.dtype for i in imgpoints]}")
print(f'object point type: {objpoints.type()}')


ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints, corrected_imgpoints, gray.shape[::-1], K, D
)
print("Intrinsic Matrix (K):")
print(K)
print("\nDistortion Coefficients (D):")
print(D)

# Save calibration data for future use
np.savez("calibration_data.npz", K=K, D=D)
