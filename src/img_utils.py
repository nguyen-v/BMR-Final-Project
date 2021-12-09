## 
# @file img_utils.py
#
# @brief Some useful functions for image processing.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import math

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Default Aruco dictionary
DEF_ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Returns the Euclidean distance between two points
#  @param   a       First point (x, y)
#  @param   b       Second point (x, y)
#  @return          Euclidean distance between the two points.
def dist(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

## Calculates the angle formed by x-axis and the line intersecting two input points.
#  @param   x1      x-coordinate of first point
#  @param   y1      y-coordinate of first point
#  @param   x2      x-coordinate of second point
#  @param   y2      y-coordinate of second point
#  @return          Angle formed by x-axis and the line intersecting two input points.
def angle_two_points(x1, y1, x2, y2):
    return math.atan2(-(y2-y1), x2-x1)