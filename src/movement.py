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
from compute_global_path import *
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

BASE_LEFT_MOTOR_SPEED = 100

BASE_RIGHT_MOTOR_SPEED = 100

# threading.Timer(0.0333,camera_feed)

# t2 = threading.Thread(target=move)

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #


def initialization():
    cam = init_camera()
    found_thymio = False
    success = False
    while (success == False):
        img, img_taken = take_picture(cam)
        if img_taken:
            M, rect_width, rect_height, map, map_enlarged, success = create_map(img, MAP_WIDTH_CELL, MAP_HEIGHT_CELL)
            rect_map = get_rectified_img(img, M, rect_width, rect_height)
        else:
            print("img not taken")
        time.sleep(0.1)
        print("map created")
    while found_thymio == False:
        thymio_pos, found_thymio = locate_thymio_camera(rect_map, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
    return M, rect_map, map, map_enlarged, cam, rect_width, rect_height

def thymio_move():
    path_found = 0
    thymio_path = []
    thymio = MyThymio()
    M, rect_map, map, map_enlarged, cam, rect_width, rect_height = initialization()

    while not path_found:
        thymio_path, path_found = global_path(thymio, cam, map_enlarged, M, rect_width, rect_height, map_enlarged)  

    for cell in range(len(thymio_path)):
        thymio_path[cell] = cell_to_xy(thymio_path[cell], MAP_WIDTH_CELL, MAP_HEIGHT_CELL, rect_width, rect_height)

    while True:
        if  state == "follow_path":
            state = follow_path(thymio, cam, thymio_path, rect_map)
        elif state == "local_avoidance":
            local_avoidance(thymio, thymio_path, cam, M, rect_width, rect_height)
            state = "follow_path"
        elif state == "End":
            thymio.stop_thymio()
            break

def global_path(thymio, cam, M, rect_width, rect_height, map_enlarged):
    img, ret_val = take_picture(cam)
    if ret_val:
        rect_map = get_rectified_img(img, M, rect_width, rect_height)
        thymio_pos, found_thymio = locate_thymio_camera(rect_map, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        goal_pos, goal_found = locate_goal_camera(rect_map, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        thymio_pos = [thymio_pos[0], thymio_pos[1]]
        if found_thymio:
            thymio_path, path_found = A_Star(thymio_pos, goal_pos, occupancy_grid, map_enlarged)
            return thymio_path, path_found

def follow_path(thymio, cam, thymio_path, rect_map):
    dist = 0
    T_s = 0.1
    kalman = KF()

    while True:
        est_dist = 0
        current_goal = thymio_path.pop(0)
        thymio_pos, theta_m, abs_v, v_m, found_thymio, found_obstacle = thymio.measurements(rect_map)
        last_angle = theta_m
        last_pos = thymio_pos

        if found_obstacle:
            state = state_machine[3]
            return state

        if found_thymio: 
            dist = calculate_distance(thymio_pos , current_goal)
            angle = calculate_orientation(thymio_pos, current_goal) #should be returned by the local function
            print(dist)
            last_angle = angle
        else:
            angle = last_angle

        da = theta_m - angle

        if da > 0:
            if da < math.pi:
                da = -da
            if da > math.pi:
                da = 2*math.pi - da
        elif da < 0:
            if da > -math.pi:
                da = -da
            if da < -math.pi:
                da = -2*math.pi - da

        thymio.rotate_thymio(-da)
        thymio.set_motor_speeds(BASE_LEFT_MOTOR_SPEED,BASE_RIGHT_MOTOR_SPEED)

        while dist >= est_dist:
            img, ret_val = take_picture(cam)
            if ret_val:
                rect_map = get_rectified_img(img, M, rect_width, rect_height)
                start_time = time.time()
                thymio_pos, theta_m, abs_v, v_m, found_thymio, found_obstacle = thymio.measurements(rect_map)
                if found_obstacle:
                    state = state_machine[3]
                    return state
                X_next = kalman.filter(thymio_pos, abs_v, T_s)
                est_dist = est_dist + calculate_distance(last_pos,X_next)
                print("X_next : {}".format(X_next))
                print("last pos : {}".format(last_pos))
                print(est_dist)
                last_pos = X_next
                end_time = time.time()
                dT_s = T_s - (end_time - start_time)
                if dT_s < 0:
                    dT_s = 0
                time.sleep(dT_s)

        thymio.stop_thymio()

        if not thymio_path:
            break
        else:
            current_goal = thymio_path.pop(0)

# state = "localization"
# thymio = MyThymio(verbose = True)
# thymio.stop_thymio()

# M, rect_map, map, map_enlarged, cam, rect_width, rect_height = initialization()
# thymio_path = ([[200,200],[400,400]])
# current_goal = [200,200]
# image = cv2.circle(rect_map, [400,400], 10, [255,0,0],-1)
# cv2.imshow('Window', rect_map)
# cv2.waitKey(1)
# follow_path(thymio, cam, thymio_path, rect_map, state)
# last_angle = 0
# img, ret_val = take_picture(cam)
# if ret_val:
#     rect_map = get_rectified_img(img, M, rect_width, rect_height)
#     thymio_pos, theta_m, abs_v, v_m, found_thymio, found_obstacle = thymio.measurements(rect_map)
#     if found_thymio: 
#         dist = calculate_distance(thymio_pos , current_goal)
#         angle = calculate_orientation(thymio_pos, current_goal) #should be returned by the local function
#         da = theta_m - angle
#         if da > 0:
#             if da < math.pi:
#                 da = -da
#             if da > math.pi:
#                 da = 2*math.pi - da
#         elif da < 0:
#             if da > -math.pi:
#                 da = -da
#             if da < -math.pi:
#                 da = -2*math.pi - da
#         print(np.rad2deg(theta_m))
#         print(np.rad2deg(angle))
#         print(np.rad2deg(-da))
#         thymio.rotate_thymio(-da)
#         time.sleep(1)

# follow_path(thymio, cam, thymio_path, rect_map, state)


