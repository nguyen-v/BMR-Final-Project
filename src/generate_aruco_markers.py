## 
# @file generate_aruco_markers.py
#
# @note  opencv-contrib-python is needed for aruco module.
# @brief Generates Aruco markers for map corner, robot and objective detection.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import os
import numpy as np
import cv2
from img_utils import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Marker size in pixels.
MARKER_SIZE_PX = 300

## Width of the marker borders.
BORDER_BITS = 1

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

## Generates an Arudo marker
#  @param id        Marker ID in the corresponding dictionary.
#  @param dict      Aruco dictionary.
def generate_aruco_marker(id, dict = DEF_ARUCO_DICT):
    aruco_dict = cv2.aruco.Dictionary_get(dict)
    marker = np.zeros((MARKER_SIZE_PX, MARKER_SIZE_PX, 1), dtype="uint8")
    cv2.aruco.drawMarker(aruco_dict, id, MARKER_SIZE_PX, marker, BORDER_BITS)
    return marker

## Generates and saves Aruco marker.
#  @param id_list   Marker IDs in the corresponding dictionary.
#  @param name      Filename.
#  @param dict      Aruco dictionary.
def get_aruco_markers(id_list, name = "", dict = DEF_ARUCO_DICT):
    dirname = os.path.dirname(__file__)
    img_path = os.path.join(dirname, '../img/markers/')

    if type(id_list) is list:
        for id in id_list:
            marker = generate_aruco_marker(id, dict)
            cv2.imwrite(img_path + name + str(id) + ".png", marker)
    else:
        marker = generate_aruco_marker(id_list, dict)
        cv2.imwrite(img_path + name + str(id_list) + ".png", marker)

## Example
# get_aruco_markers(THYMIO_ID, "thymio_marker")
# get_aruco_markers(OBJECTIVE_ID, "obj_marker")
# get_aruco_markers(MAP_CORNER_ID, "map_marker")
