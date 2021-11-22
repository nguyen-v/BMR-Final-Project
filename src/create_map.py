## 
# @file create_map.py
#
# @brief Definition of functions to create a map from an input BGR image.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from img_utils import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

dirname = os.path.dirname(__file__)
img_path = os.path.join(dirname, '../img/map_test_noisy.png')

## Set to true for testing this module.
CREATE_MAP_TEST = True

## Low threshold for red in HSV color space.
RED_THR_HSV_LOW = (0, 110, 110)

## High threshold for red in HSV color space.
RED_THR_HSV_HIGH = (15, 255,255)

## Raw image width in pixels.
RAW_IMG_WIDTH = 800

## Raw image height in pixels.
RAW_IMG_HEIGHT = 600

## Number of corners for the map boundary
NUM_MAP_CORNERS = 4

## Binary image conversion low threshold.
BIN_THR_LOW = 128

## Binary image conversion high threshold.
BIN_THR_HIGH = 255

## Binary image max value (white).
WHITE = 255

## Binary image min value (black).
BLACK = 0

## Defines how close to the 4 corners of a cell to look when checking for
#  the presence of an obstacle (1 = check right up to the edge, 0 = check center).
CHECK_CORNER_COEFF = 0.8

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Creates a map for an input BGR image.
#  @param img           Raw BGR image from camera.
#  @param map_width     Width in number of cells of the input map.
#  @param map_height    Height in number of cells of the input map.
#  @return M            Warp transform matrix for image rectification.
#  @return rect_width   Width in pixels of rectified image.
#  @return rect_height  Height in pixels of rectified image.
#  @return map          Binary map calculated from rectified image.
#  @return map_enlarged Binary map with enlarged obstacles.
#  @return success      True if maps were successfully computed.
def create_map(img, map_width, map_height, verbose = False):
    M, rect_width, rect_height, success = get_warp_matrix(img, map_width, map_height, verbose)
    if success == False:
        return [], 0, 0, [], [], success
    img_rect = get_rectified_img(img, M, rect_width, rect_height)
    # Convert to grayscale
    img_rect_gray = cv2.cvtColor(img_rect, cv2.COLOR_BGR2GRAY)
    # Convert to binary image
    (thresh, img_rect_bin) = cv2.threshold(img_rect_gray, BIN_THR_LOW, BIN_THR_HIGH, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    map = np.ones((map_height, map_width))*WHITE
    size_cell_px = rect_width/map_width
    d = int(size_cell_px/2*CHECK_CORNER_COEFF)

    for row in range(0, map_height):
        for col in range(0, map_width):
            (x, y) = cell_to_xy((row, col), map_width, map_height, rect_width, rect_height)

            # Now check 4 points near the cell corners and find the average luminance.
            # If the average is smaller than WHITE/2, it is considered black (obstacle)
            if (img_rect_bin[y+d][x+d]/4 + img_rect_bin[y+d][x-d]/4 + 
                img_rect_bin[y-d][x+d]/4 + img_rect_bin[y-d][x-d]/4) < WHITE/2:
                map[row, col] = BLACK


    # Dilate the map
    kernel = np.ones((3,3),np.uint8)
    map_enlarged = cv2.erode(map,kernel,iterations = 1)

    return M, rect_width, rect_height, map, map_enlarged, success


## Returns a the warp transform matrix for image rectification, and map dimensions.
#  @param img           Raw BGR image from camera.
#  @param map_width     Width in number of cells of the input map.
#  @param map_height    Height in number of cells of the input map.
#  @return M            Warp transform matrix for image rectification.
#  @return rect_width   Width in pixels of rectified image.
#  @return rect_height  Height in pixels of rectified image.
#  @return success      True if warp transform matrix was successfully computed.
def get_warp_matrix(img, map_width, map_height, verbose = False):
    # get dimensions
    height, width, channels = img.shape

    if verbose:
        print("Image dimensions are {} x {}".format(width, height))

    map_corners, found_pts = get_color_dots(img, RED_THR_HSV_LOW, RED_THR_HSV_HIGH, NUM_MAP_CORNERS)     
    if found_pts == False:
        return [], 0, 0, found_pts
    img_center = [0, 0]
    for corner in map_corners:
        img_center[0] = img_center[0] + corner[0]/NUM_MAP_CORNERS
        img_center[1] = img_center[1] + corner[1]/NUM_MAP_CORNERS

    # Sort corners: southmost to northmost
    map_corners = sorted(map_corners, key=lambda x: x[1], reverse=True)

    if verbose:
        for corner in map_corners:
            print("Map corner: x: {}, y: {}".format(corner[0], corner[1]))
        print("Image center is ({}, {})".format(img_center[0], img_center[1]))

    # Label the corners
    top_left = [0, 0]
    top_right = [0, 0]
    bot_left = [0, 0]
    bot_right = [0, 0]
    if map_corners[0][0] <= map_corners[1][0]:
        bot_left = map_corners[0]
        bot_right = map_corners[1]
    elif map_corners[0][0] > map_corners[1][0]:
        bot_left = map_corners[1]
        bot_right = map_corners[0]

    if map_corners[2][0] <= map_corners[3][0]:
        top_left = map_corners[2]
        top_right = map_corners[3]
    elif map_corners[2][0] > map_corners[3][0]:
        top_left = map_corners[3]
        top_right = map_corners[2]

    if verbose:
        print("Top left {}".format(top_left))
        print("Top Right {}".format(top_right))
        print("Bottom left {}".format(bot_left))
        print("Bottom right {}".format(bot_right))

    rect_width = 0 
    rect_height = 0
    # We want to have the final image bounded by the initial image dimensions
    if (map_width/map_height > RAW_IMG_WIDTH/RAW_IMG_HEIGHT):
        rect_width = RAW_IMG_WIDTH
        rect_height = int(map_height*RAW_IMG_WIDTH/map_width)
    elif (map_width/map_height <= RAW_IMG_WIDTH/RAW_IMG_HEIGHT):
        rect_width = int(map_width*RAW_IMG_HEIGHT/map_height)
        rect_height = RAW_IMG_HEIGHT
    
    if verbose:
        print("Rectified image dimensions are {} x {}".format(rect_width, rect_height))

    src_pts = np.float32([top_left, top_right, bot_left, bot_right])
    rect_pts = np.float32([[0, 0], [rect_width, 0], [0, rect_height], [rect_width, rect_height]])

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(src_pts, rect_pts)
    return M, rect_width, rect_height, True

## Returns a rectified image
#  @param img           Raw BGR image from camera.
#  @param M             Warp transform matrix for image rectification.
#  @param rect_width    Width in pixels of rectified image.
#  @return rect_img     Height in pixels of rectified image.
def get_rectified_img(img, M, rect_width, rect_height):
    return cv2.warpPerspective(img, M, (rect_width, rect_height),flags=cv2.INTER_LINEAR)

## Returns (x, y) positions from cell (row, col) postitions
#  @param cell          Cell position (row, col)
#  @param map_width     Width in number of cells of the input map.
#  @param map_height    Height in number of cells of the input map.
#  @return rect_width   Width in pixels of rectified image.
#  @return (x, y)       X-Y tuple coordinates of corresponding cell.
def cell_to_xy(cell, map_width, map_height, rect_width, rect_height):
    return (int((rect_width/map_width)*(cell[1]+1/2)), int((rect_height/map_height)*(cell[0]+1/2)))

# ========================================================================== #
#  Testbenches                                                               # 
# ========================================================================== #

if CREATE_MAP_TEST:
    # read image
    img = cv2.imread(img_path)
    plt.figure()
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Raw image")

    # Initialize map
    M, rect_width, rect_height, map, map_enlarged, success = create_map(img, 11, 7, verbose = True)
    if success:
        plt.figure()
        plt.imshow(map_enlarged, origin = 'lower')
        plt.title("Map enlarged")
        plt.gca().invert_yaxis()
        plt.figure()
        plt.imshow(map, origin = 'lower')
        plt.title("Original Map")
        plt.gca().invert_yaxis()
        # We can now get the rectified image using the warp transform matrix
        # Separating the processes allow us to recalculate quickly the rectified map
        # without having to recalculate the warp transform matrix (assuming fixed camera).
        img_rect = get_rectified_img(img, M, rect_width, rect_height)
        plt.figure()
        plt.imshow(cv2.cvtColor(img_rect, cv2.COLOR_BGR2RGB))
        plt.title("Rectified image")
        plt.show()
    else:
        print("Map was not successfully computed.")