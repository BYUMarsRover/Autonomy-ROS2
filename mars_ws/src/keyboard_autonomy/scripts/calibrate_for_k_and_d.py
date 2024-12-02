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

# Reshape object points and image points to match cv2.fisheye.calibrate() requirements
objpoints_reshaped = [points.reshape(-1, 1, 3) for points in objpoints]
imgpoints_reshaped = [points.reshape(-1, 1, 2) for points in imgpoints]

# Perform calibration
K = np.zeros((3, 3))
D = np.zeros((4, 1))

print(f"objpoints shape: {[o.shape for o in objpoints_reshaped]}")
print(f"objpoints dtype: {[o.dtype for o in objpoints_reshaped]}")
print(f"imgpoints shape: {[i.shape for i in imgpoints_reshaped]}")
print(f"imgpoints dtype: {[i.dtype for i in imgpoints_reshaped]}")

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints_reshaped, imgpoints_reshaped, gray.shape[::-1], K, D
)

print("Intrinsic Matrix (K):")
print(K)
print("\nDistortion Coefficients (D):")
print(D)

# Save calibration data for future use
np.savez("calibration_data.npz", K=K, D=D)
