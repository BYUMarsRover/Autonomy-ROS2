import cv2
import numpy as np
import glob
import json

CHECKERBOARD = (10, 7)
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

# Prepare object points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Initialize storage for points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane
image_shape = None  # Placeholder for image shape

# Load all calibration images
images = glob.glob("calibration_images/*.jpg")
for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Failed to load image: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_shape is None:
        image_shape = gray.shape[::-1]  # Set image shape based on the first valid image

    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
        imgpoints.append(corners)
    else:
        print(f"Chessboard corners not found in image: {fname}")

# Verify points before calibration
if len(objpoints) == 0 or len(imgpoints) == 0:
    raise ValueError("No valid chessboard corners found. Ensure images are correct.")

# Perform fisheye calibration
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]

rms, _, _, _, _ = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    image_shape,
    K,
    D,
    rvecs,
    tvecs,
    calibration_flags,
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
)

print(f"RMS error: {rms}")
print("Intrinsic Matrix (K):")
print(K)
print("Distortion Coefficients (D):")
print(D)

# Save calibration data
data = {
    "image_shape": image_shape,
    "K": K.tolist(),
    "D": D.tolist()
}
with open("fisheye_calibration_data.json", "w") as f:
    json.dump(data, f)
