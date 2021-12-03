## 
# @file movement.py
#
# @brief Definition of class and functions to manipulate Thymio

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import threading
import cv2
import os
import time
import numpy as np
import math

from create_map import *
from MyThymio import *
from camera import *
from local_navigation import *
from utils import *
from Kalman import kalman_filter as KF

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../img/camera_shot.jpg')

state_machine = ["initialization", "localization", "follow_path", "local_avoidance"]
threads = []
img_ready = False

ROTATION_THRESHOLD = 15 # degrees

HORZ_PROX_THRESHOLD = [4000, 3000, 2000, 3000, 4000]



# threading.Timer(0.0333,camera_feed)

# t2 = threading.Thread(target=move)

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

def initialization():
    cam = init_camera()
    img = take_picture(cam)
    M, rect_width, rect_height, map, map_enlarged, success = create_map(img, MAP_WIDTH_CELL, MAP_HEIGHT_CELL)
    rect_map, rect_width, rect_height, success  = get_warp_matrix(img, MAP_WIDTH_CELL, MAP_HEIGHT_CELL)
    # Calculate Global Path (Function required)
    return M, rect_map, map, map_enlarged, cam, img

def thymio_move():

    thymio = MyThymio()
    thymio_path = []
    M, rect_map, map, map_enlarged, cam, img = initialization()
    state == "localization"

    while True:
        if state == "localization":
            nodes = global_path(thymio_pos, goal_pos, map_enlarged, state)
        elif state == "follow_path":
            state = follow_path(thymio, cam, thymio_path, rect_map)
        elif state == "local_avoidance":
            local_avoidance(thymio, thymio_path, cam, M, rect_width, rect_height)
        elif state == "End":
            break

def follow_path(thymio, cam, thymio_path, rect_map, state):

    last_angle = 0
    turn_direction = 1
    est_dist = 0
    start_time = time.time()
    rotate = 1
    kalman = KF()
    current_goal = thymio_path.pop(1)

    while True:
        thymio_pos, theta_m, abs_v, v_m, found_thymio, found_obstacle = thymio.measurements(rect_map)
        end_time = time.time()
        print(end_time)
        new_T_s = end_time - start.time

        if found_obstacle:
            state == state_machine[3]
            return state
    
        if found_thymio: 
            dist = calculate_distance(thymio_pos , current_goal)
            angle = calculate_orientation(thymio_pos, current_goal) #should be returned by the local function
            last_angle = angle
        else:
            angle = last_angle

        da = theta_m - angle

        if da > 0:
            if da > math.pi/2:
                angle = -angle
        elif da < 0:
            if da > -math.pi/2:
                angle = -angle

        thymio.rotate_thymio(angle)

        while dist >= est_dist:
            thymio_pos, theta_m, abs_v, v_m, found_thymio, found_obstacle = thymio.measurements(rect_map)
            X_next = kalman.filter(thymio_pos, abs_v, new_T_s)
            est_dist = est_dist + calculate_distance(last_pos,X_next)
            last_pos = X_next

        current_goal = thymio_path.pop(1)

thymio = MyThymio()
M, rect_map, map, map_enlarged, cam, img = initialization()
follow_path(thymio, cam, thymio_path, rect_map, state)

