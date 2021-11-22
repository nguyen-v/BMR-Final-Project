## 
# @file img_utils.py
#
# @brief Some useful functions for image processing.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import matplotlib.pyplot as plt
import numpy as np

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Returns a list of (x, y) positions of dots that satisfy the low and
#  high HSV thresholds. Number of points to return can be specified.
#  @param img           Raw BGR image from camera.
#  @param HSV_THR_LOW   Low threshold in HSV space.
#  @param HSV_THR_HIGH  High threshold in HSV space.
#  @param exp_num_pts   Expected number of points (optional)
#  @return dots         A list of (x, y) positions of dots. The size of the list is the same
#                       as the number of expected points if the number of found points is greater than
#                       the number of expected points. Points are sorted by size from biggest to smallest.
#  @return found_pts    True if the number of found points is greater than the expected number of points.
def get_color_dots(img, HSV_THR_LOW, HSV_THR_HIGH, exp_num_pts = None):
    # convert to hsv space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # extract a certain color
    mask = cv2.inRange(hsv, HSV_THR_LOW, HSV_THR_HIGH)
    # erode mask to avoid noise
    kernel = np.ones((2,2),np.uint8)
    mask = cv2.erode(mask,kernel,iterations = 3)
    plt.figure()
    plt.imshow(mask)
    # extract contours of dots
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = list(contours)
    # Sort from bigger dots (longest contour) to smaller dots (smallest contours)
    contours.sort(key = len, reverse = True)
    dots = []
    # find centroid of dots
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        dots.append((cX, cY))
    
    if exp_num_pts != None:
        if len(dots) >= exp_num_pts:
            return dots[0:exp_num_pts], True
        else:
            return [], False
    return dots, True