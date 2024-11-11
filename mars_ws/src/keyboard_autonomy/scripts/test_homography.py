import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

'''
:author: Nelson Durrant
:date: November 2024

This script is used to test the OpenCV keyboard homography calculation. 
It uses the SIFT algorithm to find keypoints and descriptors in both images, 
and then uses the FLANN algorithm to find matches between the descriptors. 
If enough matches are found, the homography is calculated and displayed.
'''

start_time = time.time()

# Minimum number of matches required
MIN_MATCH_COUNT = 10

# Load the images using OpenCV
img1 = cv2.imread("../images/keyboard_cropped.jpg")
img2 = cv2.imread("../images/keyboard1.jpg")

image_read_time = time.time()

# Initiate SIFT detector and find the keypoints and descriptors
# surf = cv2.xfeatures2d.SURF_create(400)  # The threshold value can be adjusted.
# kp1, des1 = surf.detectAndCompute(img1, None)
# kp2, des2 = surf.detectAndCompute(img2, None)
sift = cv2.SIFT_create(nfeatures=500, nOctaveLayers=2, contrastThreshold=0.04, edgeThreshold=10)
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

detect_and_compute_time = time.time()

# Set FLANN parameters (TODO: What are these?)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

parameter_dictionary_time = time.time()

# Create the FLANN matcher and find matches (best two matches for each descriptor)
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)

matches_calculated_time = time.time()

# Store all the good matches using Lowe's ratio test
# Check to make sure that the distance to the closest match is sufficiently
# less than the distance to the second closest match
good_matches = []
for m, n in matches:
    if m.distance < 0.7 * n.distance:
        good_matches.append(m)

# If enough matches are found, calculate the homography
if len(good_matches) >= MIN_MATCH_COUNT:


    # Get the keypoints from the good matches
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1, 1, 2)

    # Calculate the homography matrix and mask
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matchesMask = mask.ravel().tolist()

    homography_calculated_time = time.time()
    # Plot and display the results
    h, w = img1.shape[0:2]
    pts = np.float32([[0, 0],[0, h-1],[w-1, h-1],[w-1, 0]]).reshape(-1, 1, 2) # get the corners
    outline = cv2.perspectiveTransform(pts, M) # transform the corners of the first image onto the second
    img2 = cv2.polylines(img2, [np.int32(outline)], True, 255, 3, cv2.LINE_AA) # draw the box
    draw_params = dict(
            singlePointColor = None,
            matchesMask = matchesMask,
            flags = 2)
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, **draw_params)
    plt.imshow(img3, 'gray')
    plt.show()

else:
    print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))



end_time = time.time()

elapsed_time = end_time - start_time

print(f"time for image reading: {image_read_time-start_time}")
print(f"time for detecting and computing SIFT keypoints and descriptors: {detect_and_compute_time-image_read_time}")
print(f"time for creating the dictionaries: {parameter_dictionary_time-detect_and_compute_time}")
print(f"time for calling FlannBasedMatcher and knnMatch: {matches_calculated_time-parameter_dictionary_time}")
print(f"time for calculating the homography matrix and mask: {homography_calculated_time-matches_calculated_time}")


