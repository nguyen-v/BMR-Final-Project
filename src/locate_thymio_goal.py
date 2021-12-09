## 
# @file locate_thymio_goal.py
#
# @brief Definition functions to locate Thymio and objective.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import math 
import numpy as np

## Custom modules
from img_utils import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Thymio marker ID.
THYMIO_ID = 5

## Objective ID.
OBJECTIVE_ID = 6

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #


#thymio's rotaion point is blue, thymio's "direction point" is purple, goal is green;

# transform (x,y) coordinates into grid coordinates
#  @param       coords          cartesian coordinates
#  @param       map_size        size of the map in pixels (size(x),size(y))
#                               cartesian will return (x,y) coordinates
#                               grid will return (column, line) coordinates
#  @param       grid_size       size of the grid (nb_columns,nb_lines)
#  @return      grid_coords     pos in the grid
def cartesian_to_grid(coords,map_size,grid_size):
    grid_coords = (math.floor(coords[1]*grid_size[1]/map_size[1]), math.floor(coords[0]*grid_size[0]/map_size[0]))
    return grid_coords


## Returns position of the thymio if found
#  @param       rectified_img   Array containing each pixels of the rectified image from the camera 
#  @param       coord_type      string variable that can contain "cartesian" or "grid"
#                               cartesian will return (x,y) coordinates
#                               grid will return (column,line) coordinates
#  @param       grid_size       size of the grid (nb_colums,nb_lines)
#  @return      thymio_pose     gives thymio (x,y, angle) or (column,line, angle) coordinates
#  @return      found_thymio    (bool) returns true if thymio was found, false otherwise
#  @note                        Adapted from https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def locate_thymio_camera(rectified_img,coord_type, grid_size):

    aruco_dict = cv2.aruco.Dictionary_get(DEF_ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(rectified_img, aruco_dict, parameters=aruco_params)

    thymio_pose = []
    if len(corners) >= 1:
        ids = ids.flatten()
        for (corner, id) in zip(corners, ids):
            if id == THYMIO_ID:

                # extract the marker corners (which are always returned in 
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = corner.reshape((4, 2))
                (top_left, top_right, bot_right, bot_left) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bot_right = (int(bot_right[0]), int(bot_right[1]))
                bot_left = (int(bot_left[0]), int(bot_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((top_left[0] + bot_right[0] + top_right[0] + bot_left[0]) / 4.0)
                cY = int((top_left[1] + bot_right[1] + top_right[1] + bot_left[1]) / 4.0)
                angle = math.atan2(-((top_left[1]+top_right[1])/2 - (bot_left[1]+bot_right[1])/2), 
                                     (top_left[0]+top_right[0])/2 - (bot_left[0]+bot_right[0])/2)

                if(coord_type == 'cartesian'):
                    thymio_pose = np.append([cX, cY], angle)    
                    return thymio_pose, True
                else:
                    map_size = (np.size(rectified_img, 1),np.size(rectified_img, 0))
                    thymio_coords  = cartesian_to_grid((cX, cY), map_size, grid_size)
                    thymio_pose = np.append(thymio_coords,angle)
                    return thymio_pose, True           
    return [], False



## Returns position of the goal if found
#  @param       rectified_img   Array containing each pixels of the rectified image from the camera 
#  @param       coord_type      string variable that can contain "cartesian" or "grid"
#                               cartesian will return (x,y) coordinates
#                               grid will return (column,line) coordinates
#  @param       grid_size       size of the grid (nb_columns,nb_lines)
#  @return      goal_coords     gives goal (x,y) or (column,line, angle) coordinates
#  @return      found_goal      (bool) returns true if goal was found, false otherwise 
#  @note                        Adapted from https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def locate_goal_camera(rectified_img,coord_type, grid_size):
<<<<<<< HEAD
    
    goal_coords, found_goal = get_color_dots(rectified_img, GREEN_THR_HSV_LOW, GREEN_THR_HSV_HIGH, 1)
    
    if(found_goal):
        goal_coords = goal_coords[0]
        if(coord_type == 'cartesian'):
            return goal_coords, True
        else:
            map_size = (np.size(rectified_img, 1),np.size(rectified_img, 0))
            goal_coords  = cartesian_to_grid(goal_coords,map_size,grid_size)
            return goal_coords, True   
    else:
        return [], False 
=======

    aruco_dict = cv2.aruco.Dictionary_get(DEF_ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(rectified_img, aruco_dict, parameters=aruco_params)
    obj_pos = []

    if len(corners) >= 1:
        ids = ids.flatten()
        for (corner, id) in zip(corners, ids):
            if id == OBJECTIVE_ID:

                # extract the marker corners (which are always returned in 
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = corner.reshape((4, 2))
                (top_left, top_right, bot_right, bot_left) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bot_right = (int(bot_right[0]), int(bot_right[1]))
                bot_left = (int(bot_left[0]), int(bot_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((top_left[0] + bot_right[0] + top_right[0] + bot_left[0]) / 4.0)
                cY = int((top_left[1] + bot_right[1] + top_right[1] + bot_left[1]) / 4.0)
                
                if(coord_type == 'cartesian'):
                    obj_pos = [cX, cY]  
                    return obj_pos, True
                else:
                    map_size = (np.size(rectified_img, 1),np.size(rectified_img, 0))
                    obj_pos  = cartesian_to_grid((cX, cY), map_size, grid_size)
                    return obj_pos, True           
    return [], False
>>>>>>> master

    