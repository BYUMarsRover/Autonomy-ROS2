import cv2
import numpy as np
from matplotlib import pyplot as plt

'''
:author: Nelson Durrant
:date: November 2024

This script is used to test the OpenCV keyboard homography calculation. 
It uses the SIFT algorithm to find keypoints and descriptors in both images, 
and then uses the FLANN algorithm to find matches between the descriptors. 
If enough matches are found, the homography is calculated and a plot is displayed
showing the matches and the homography outline.
'''

# Minimum number of matches required
MIN_MATCH_COUNT = 10

# Load the images using OpenCV
img1 = cv2.imread("keyboard.jpg")
img2 = cv2.imread("keyboard.jpg")

# Initiate SIFT detector and find the keypoints and descriptors
sift = cv2.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

print("SIFT keypoints and descriptors found")

# Set FLANN parameters (TODO: What are these?)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

# Create the FLANN matcher and find matches
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)

print("FLANN matches found")

# Store all the good matches using Lowe's ratio test (TODO: What is this doing?)
good_matches = []
for m, n in matches:
    if m.distance < 0.7 * n.distance:
        good_matches.append(m)

# If enough matches are found, calculate the homography
if len(good_matches) >= MIN_MATCH_COUNT:

    # Get the keypoints from the good matches
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1, 1, 2)

    # Calculate the homography
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matchesMask = mask.ravel().tolist()

    # Plot and display the results (TODO: What is this doing?)
    h, w = img1.shape[0:2]
    pts = np.float32([[0, 0],[0, h-1],[w-1, h-1],[w-1, 0]]).reshape(-1, 1, 2)
    outline = cv2.perspectiveTransform(pts, M)
    img2 = cv2.polylines(img2, [np.int32(outline)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(
            singlePointColor = None,
            matchesMask = matchesMask, # draw only inliers
            flags = 2)
    img3 = cv2.drawMatches(img1,kp1,img2,kp2,good_matches,None,**draw_params)
    plt.imshow(img3, 'gray')
    plt.show()

    print("Homography calculated")

else:
    print("Not enough matches are found - %d/%d" % (len(good_matches), MIN_MATCH_COUNT))
