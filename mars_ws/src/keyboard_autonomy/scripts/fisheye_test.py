import cv2
import numpy as np

# Load the fisheye image
img = cv2.imread("calibration_images/2024-11-25-090806.jpg")
h, w = img.shape[:2]

# Camera matrix (K): Approximation
K = np.array([[240.0742270720785, 0.0, 303.87271958823317], [0.0, 240.2956790772891, 242.63742883611513], [0.0, 0.0, 1.0]], dtype=np.float32)

# Distortion coefficients (D)
D = np.array([[-0.032316597779098725],
              [-0.014192392170667197],
              [0.004485507618487958],
              [-0.0007679973472430706]], dtype=np.float32)

# Adjust K to prevent zooming
# TODO: decide if we need this
scaled_K = K.copy()
scaled_K[0, 0] *= 0.8  # Reduce focal length in x-direction
scaled_K[1, 1] *= 0.8  # Reduce focal length in y-direction

# Undistortion map
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), scaled_K, (w, h), cv2.CV_16SC2
)

# Undistort the image
undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

# Show and save the undistorted image
cv2.imshow("Original", img)
cv2.imshow("Undistorted", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("undistorted_img.png", undistorted_img)
