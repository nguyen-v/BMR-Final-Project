## 
# @file create_map.py
#
# @brief Definition of functions to create a map from an input BGR image.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import numpy as np
from img_utils import *
from camera import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Low threshold for red in HSV color space.
RED_THR_HSV_LOW = (0, 100, 100)

## High threshold for red in HSV color space.
RED_THR_HSV_HIGH = (5, 255,255)

## Raw image width in pixels.
RAW_IMG_WIDTH = 800

## Raw image height in pixels.
RAW_IMG_HEIGHT = 600

## Number of corners for the map boundary.
NUM_MAP_CORNERS = 4

## Map width in number of cells.
MAP_WIDTH_CELL = 33

## Map height in number of cells.
MAP_HEIGHT_CELL = 24

## Binary image conversion low threshold.
BIN_THR_LOW = 128

## Binary image conversion high threshold.
BIN_THR_HIGH = 255

## Map obstacle luminance threshold.
OBS_LUM_THR = 100

## Binary image max value (white).
WHITE = 255

## Binary image min value (black).
BLACK = 0

## Defines how close to the 4 corners of a cell to look when checking for
#  the presence of an obstacle (1 = check right up to the edge, 0 = check center).
CHECK_CORNER_COEFF = 0.3

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

    # Remove aruco tags from image
    img_rect = remove_aruco_tags(img_rect)

    # Convert to grayscale
    img_rect_gray = cv2.cvtColor(img_rect, cv2.COLOR_BGR2GRAY)
    # Convert to binary image
    (thresh, img_rect_bin) = cv2.threshold(img_rect_gray, BIN_THR_LOW, BIN_THR_HIGH, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # (thresh, img_rect_bin) = cv2.threshold(img_rect_gray, BIN_THR_LOW, BIN_THR_HIGH, cv2.THRESH_BINARY)

    map = np.ones((map_height, map_width))*BLACK
    size_cell_px = rect_width/map_width
    d = int(size_cell_px/2*CHECK_CORNER_COEFF)

    for row in range(0, map_height):
        for col in range(0, map_width):
            (x, y) = cell_to_xy((row, col), map_width, map_height, rect_width, rect_height)

            # Now check 4 points near the cell corners and find the average luminance.
            # If the average is smaller than OBS_LUM_THR, it is considered black (obstacle)
            if (img_rect_bin[y+d][x+d]/4 + img_rect_bin[y+d][x-d]/4 + 
                img_rect_bin[y-d][x+d]/4 + img_rect_bin[y-d][x-d]/4) < OBS_LUM_THR:
                map[row, col] = WHITE


    # Dilate the map
    kernel = np.ones((3,3),np.uint8)
    map_enlarged = cv2.dilate(map,kernel,iterations = 1)

    return M, rect_width, rect_height, map, map_enlarged, success


def remove_aruco_tags(img_rect):
    aruco_dict = cv2.aruco.Dictionary_get(DEF_ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img_rect, aruco_dict, parameters=aruco_params)
    if len(corners) > 0:
        for corner in corners:
            # Extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
            corners = corner.reshape((4, 2))
            (top_left, top_right, bot_right, bot_left) = corners
            # Convert each of the (x, y)-coordinate pairs to integers
            top_right = (int(top_right[0]), int(top_right[1]))
            bot_right = (int(bot_right[0]), int(bot_right[1]))
            bot_left = (int(bot_left[0]), int(bot_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))

            # Draw a white square over aruco tags. This is to avoid having them detected as obstacles.
            cv2.fillPoly(img_rect, pts = [np.array([top_left, top_right, bot_right, bot_left])], color = (WHITE,WHITE,WHITE))

    return img_rect


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

    # map_corners, found_pts = get_color_dots(img, RED_THR_HSV_LOW, RED_THR_HSV_HIGH, NUM_MAP_CORNERS, is_red = True)
    # map_corners, found_pts = get_map_corners(img)     
    top_left, top_right, bot_left, bot_right, found_pts = get_map_corners(img)     
    if found_pts == False:
        return [], 0, 0, found_pts

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

## Returns list of coordinates of map corner markers
#  @param       img     input raw image
#  @return      A list of (x, y) positions of map corners.
#  @note        https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def get_map_corners(img):
    aruco_dict = cv2.aruco.Dictionary_get(DEF_ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
    map_corners = []
    if len(corners) >= NUM_MAP_CORNERS:
        ids = ids.flatten()
        for (corner, id) in zip(corners, ids):
            if id in MAP_CORNER_ID:
                # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
                corners = corner.reshape((4, 2))
                (top_left, top_right, bot_right, bot_left) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bot_right = (int(bot_right[0]), int(bot_right[1]))
                bot_left = (int(bot_left[0]), int(bot_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
                # get true map corners
                marker_corner = []
                if id == MAP_CORNER_ID[0]:
                    marker_corner = bot_right
                elif id == MAP_CORNER_ID[1]:
                    marker_corner = bot_left
                elif id == MAP_CORNER_ID[2]:
                    marker_corner = top_left
                elif id == MAP_CORNER_ID[3]:    
                    marker_corner = top_right
                map_corners.append(marker_corner)

        if len(map_corners) == NUM_MAP_CORNERS:
            img_center = [0, 0]
            for corner in map_corners:
                img_center[0] = img_center[0] + corner[0]/NUM_MAP_CORNERS
                img_center[1] = img_center[1] + corner[1]/NUM_MAP_CORNERS

            # Sort corners: southmost to northmost
            map_corners = sorted(map_corners, key=lambda x: x[1], reverse=True)

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
            return top_left, top_right, bot_left, bot_right, True
        else:
            return [0, 0], [0, 0], [0, 0], [0, 0], False
    else:
        return [0, 0], [0, 0], [0, 0], [0, 0], False

## Initializes the map.
#  @param cam           Camera instance
#  @return M            Warp transform matrix for image rectification.
#  @return rect_width   Width in pixels of rectified image.
#  @return rect_height  Height in pixels of rectified image.
#  @return map          Binary map calculated from rectified image.
#  @return map_enlarged Binary map with enlarged obstacles.
def init_map(cam):
    map_created = False
    M = []
    rect_width = 0
    rect_height = 0 
    map = []
    map_enlarged = []
    map_created = False
    while not map_created:
        img, img_taken = take_picture(cam)
        if img_taken:
            M, rect_width, rect_height, map, map_enlarged, map_created = create_map(img, MAP_WIDTH_CELL, MAP_HEIGHT_CELL)
    return M, rect_width, rect_height, map, map_enlarged


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

# # MAP INITIALIZATION EXAMPLE
# cam = init_camera()
# M, rect_width, rect_height, map, map_enlarged = init_map(cam)
# plt.figure()
# plt.imshow(map_enlarged, origin = 'lower', cmap = 'Greys', interpolation = 'nearest')
# plt.title("Map enlarged")
# plt.gca().invert_yaxis()
# plt.figure()
# plt.imshow(map, origin = 'lower', cmap = 'Greys', interpolation = 'nearest')
# plt.title("Original Map")
# plt.gca().invert_yaxis()

# img, img_taken = take_picture(cam)
# if img_taken:
#     img_rect = get_rectified_img(img, M, rect_width, rect_height)
#     plt.figure()
#     plt.imshow(cv2.cvtColor(img_rect, cv2.COLOR_BGR2RGB))
#     plt.title("Rectified image")
#     plt.show()
