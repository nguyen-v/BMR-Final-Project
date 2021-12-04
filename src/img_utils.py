## 
# @file img_utils.py
#
# @brief Some useful functions for image processing.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import numpy as np
import matplotlib.pyplot as plt

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

HSV_MAX_HUE = 180

## Map corner marker IDs.
MAP_CORNER_ID = [1, 2, 3, 4]

## Thymio marker ID.
THYMIO_ID = 5

## Objective ID.
OBJECTIVE_ID = 6

## Default Aruco dictionary
DEF_ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Returns a list of (x, y) positions of dots that satisfy the low and
#  high HSV thresholds. Number of points to return can be specified.
#  @param img           Raw BGR image from camera.
#  @param HSV_THR_LOW   Low threshold in HSV space.
#  @param HSV_THR_HIGH  High threshold in HSV space.
#  @param exp_num_pts   Expected number of points (optional)
#  @param is_red        Set to true if color to filter is red. This is a special case as red's hue lie between
#                       0-10 and 170-180 (in OpenCV)
#  @return dots         A list of (x, y) positions of dots. The size of the list is the same
#                       as the number of expected points if the number of found points is greater than
#                       the number of expected points. Points are sorted by size from biggest to smallest.
#  @return found_pts    True if the number of found points is greater than the expected number of points.
def get_color_dots(img, HSV_THR_LOW, HSV_THR_HIGH, exp_num_pts = None, is_red = False):
    # convert to hsv space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # extract a certain color
    mask = cv2.inRange(hsv, HSV_THR_LOW, HSV_THR_HIGH)
    mask_h = []
    if is_red:
        mask_h = cv2.inRange(hsv, (HSV_MAX_HUE - HSV_THR_HIGH[0], HSV_THR_LOW[1], HSV_THR_LOW[2]), (HSV_MAX_HUE, HSV_THR_HIGH[1], HSV_THR_HIGH[2]))
    # erode mask to avoid noise
    if is_red:
        mask = mask + mask_h
    cv2.imshow('mask', mask)
    cv2.waitKey(1)
    kernel = np.ones((2,2),np.uint8)
    mask = cv2.erode(mask,kernel,iterations = 3)
    # plt.figure()
    # plt.imshow(mask)
    # plt.show()
    # cv2.imshow('mask', mask)
    # cv2.waitKey(1)
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
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            dots.append((cX, cY))
        except ZeroDivisionError:
            pass
    
    if exp_num_pts != None:
        if len(dots) >= exp_num_pts:
            return dots[0:exp_num_pts], True
        else:
            return [], False
    return dots, True