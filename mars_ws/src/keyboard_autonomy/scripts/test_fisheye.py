import cv2
import numpy as np

# Load the fisheye image
img = cv2.imread("img.png")
h, w = img.shape[:2]

# Camera matrix (K): Approximation
K = np.array([[700, 0, w / 2],  # fx, 0, cx
              [0, 700, h / 2],  # 0, fy, cy
              [0, 0, 1]], dtype=np.float32)

# Distortion coefficients (D): Example with strong distortion
D = np.array([-0.5, 0.2, 0, 0], dtype=np.float32)  # Use more extreme values


# Undistortion map
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)

# Undistort the image
undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

# Show and save the undistorted image
cv2.imshow("Original", img)
cv2.imshow("Undistorted", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("undistorted_img.png", undistorted_img)
