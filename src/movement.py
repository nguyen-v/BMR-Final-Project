## 
# @file thymio_utils.py
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

import create_map as mapping
import thymio_utils as sens
from kalman import kalman_filter as KF

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../img/camera_shot.jpg')

state_machine = ["initialization", "localization", "follow_path", "local_avoidance"]
threads = []
img_ready = False

MAP_WIDTH = 640

MAP_LENGTH = 480

ROTATION_THRESHOLD = 15 # degrees

# threading.Timer(0.0333,camera_feed)

# t2 = threading.Thread(target=move)

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

def take_picture(cam):
        ret_val,img = cam.read()
        # print("ret_val : {}, is_opened : {}.".format(ret_val,cam.isOpened()))
        if ret_val:
            img_ready()
            #cv2.imshow('sus', img)
            # cv2.waitKey(1)
            save_camera_img(img)    
            return img

def img_ready():
    global img_ready 
    img_ready = True

def init_camera(width = MAP_WIDTH, height = MAP_LENGTH):
    cam=cv2.VideoCapture(0 + cv2.CAP_DSHOW)
    cam.set(3,width)
    cam.set(4,height)
    return cam

def save_camera_img(img):
    cv2.imwrite(filename, img)

def initialization():
    cam = init_camera()
    # img_ready()
    img = take_picture(cam)
    binary_map, rect_width, rect_height, map, map_enlarged, success = mapping.create_map(img, MAP_WIDTH, MAP_LENGTH)
    rect_map, rect_width, rect_height, success  = mapping.get_warp_matrix(img, MAP_WIDTH, MAP_LENGTH)
    # Locate objective (Function required)
    # Locate Thymio (Function required)
    # Calculate Global Path (Function required)
    return binary_map, rect_map, map, map_enlarged

def follow_path(nodes, thymio_init_pos):

    last_angle = 0
    turn_direction = 1
    kalman = KF()
    last_pos = thymio_init_pos

    for i in range(len(nodes)):
        dist = calculate_distance(thymio_init_pos,nodes[i])
        est_dist = 0

        if camera_available(): # Please someone implement this function
            angle = calculate_orientation(thymio_x, thymio_y, obj_x, obj_y) #should be returned by the local function
            last_angle = angle
        else:
            angle = last_angle

        a_thymio = calculate_obj_angle(thymio_pos)
        a_obj = calculate_obj_angle(obj_pos)

        if a_thymio - a_obj > 0:
            if a_thymio - a_obj <= math.pi/2:
                turn_direction = 1
            if a_thymio - a_obj > math.pi/2:
                turn_direction = -1
        elif a_thymio - a_obj < 0:
            if a_thymio - a_obj <= -math.pi/2:
                turn_direction = 1
            if a_thymio - a_obj > -math.pi/2:
                turn_direction = -1

        sens.thymio_rotate(angle, turn_direction)
        #KF.save_input_control(-vx, -vy) #save input controls, stop the motor

        while dist >= est_dist:
            X_next = KF.kalman_filter()
            est_dist = est_dist + calculate_distance(last_pos,X_next)
            last_pos = X_next

def thymio_move():
    state == "localization"
    binary_map, rect_map, map, map_enlarged = initialization()
    while True:
        if state == "localization":
            nodes = global_path(map_enlarged, state)
        elif state == "follow_path":
            follow_path(nodes, state)
        elif state == "local_avoidance":
            local_avoidance(state)
        elif state == "End":
            break

def local_avoidance():
    something = 1
    return something

# Result angle is between pi and -pi
def calculate_orientation(thymio_x, thymio_y, obj_x, obj_y):
    return math.atan2(obj_y-thymio_y, obj_x-thymio_x)

def calculate_obj_angle(thymio_pos):
    return math.atan2(thymio_pos[1],thymio_pos[0])

def calculate_distance(thymio_pos, obj_pos):
    return math.sqrt((thymio_pos[1]-obj_pos[1]) ** 2  + (thymio_pos[0] - obj_pos[0]) ** 2)