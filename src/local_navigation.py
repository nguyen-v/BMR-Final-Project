## 
# @file local_navigation.py
#
# @brief Local navigation for Thymio.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

from MyThymio import *
from locate_thymio_goal import *
from thymio_connection import *
from camera import *
from create_map import *
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

dirname = os.path.dirname(__file__)
img_path = os.path.join(dirname, '../img/test_points4.png')

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Number of proximity sensor values.
NUM_PROX_VALUES = 7

## Number of ground sensors values.
NUM_GND_SENS = 2

# get measurements from front prox sensors
# get position and orientation of Thymio
# get objective from global path
# calculate vector and associated weights
# calcultate weights from prox sensors

# purple is front of thymio
# function gives center of rotation (blue point)

# def local_avoidance(thymio):
#     thymio_pose, found_thymio = locate_thymio_camera(img, 'cartesian', (11, 7))

## Left weights for local obstacle avoidance, taken from Exercise Session 3
WEIGHTS_LEFT_PROX = [40,  20, -20, -20, -40,  30, -10]

## Right weights for local obstacle avoidance, taken from Exercise Session 3
WEIGHTS_RIGHT_PROX = [-40, -20, -20,  20,  40, -10, 30]

## Left weights for fixed obstacles avoidance.
WEIGHTS_LEFT_GND = [50, -20]

## Right weights for fixed obstacles avoidance.
WEIGHTS_RIGHT_GND = [-20, 50]

## Scale factor for proximity sensors.
PROX_SCALE = 200

## Scale factor for ground sensors.
GND_SCALE = 200

## Scale factor for motors.
MOTOR_SCALE = 15

## Objective attractiveness coefficient.
OBJ_ATT_COEFF = 50

## Objective base attractiveness
OBJ_ATT_BASE = 50

## Local avoidance threshold distance to local objective (in px).
LOC_DIST_THR = 20

x = np.zeros(NUM_PROX_VALUES)

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Avoids a local obstacle (not detected by the camera)
#  @param       thymio          Thymio instance.
#  @return      has_avoided     True if it has avoided the obstacle.
def local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height):

    img = take_picture(cam)
    rect_img = get_rectified_img(img, M, rect_width, rect_height)
    thymio_pose, found_thymio = locate_thymio_camera(rect_img, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))

    if found_thymio:
        y = np.zeros(2)

        # Local obstacles contribution (repulsive)
        front_prox = np.divide(thymio.get_prox_horizontal(), PROX_SCALE)
        for i in range(NUM_PROX_VALUES):
            y[0] = y[0] + front_prox[i] * WEIGHTS_LEFT_PROX[i]
            y[1] = y[1] + front_prox[i] * WEIGHTS_RIGHT_PROX[i]
        
        # Local objective contribution (attractive)
        a_th_obj = angle_two_points(thymio_pose[0], thymio_pose[1], obj_pos[0], obj_pos[1])
        da = thymio_pose[2] - a_th_obj
        y[0] = y[0] + OBJ_ATT_BASE + da/math.pi*OBJ_ATT_COEFF
        y[1] = y[1] + OBJ_ATT_BASE - da/math.pi*OBJ_ATT_COEFF

        # # Map obstacles contribution (repulsive)
        # gnd_sens = np.divide(thymio.get_gnd_sensors(), GND_SCALE)
        # we have to put a threshold on ground sensors if global obstacles have no gradient
        # for i in range(NUM_GND_SENS):
        #     y[0] = y[0] + gnd_sens[i] * WEIGHTS_LEFT_GND[i]
        #     y[1] = y[1] + gnd_sens[i] * WEIGHTS_RIGHT_GND[i]

        thymio.set_motor_left_speed(int(y[0]/MOTOR_SCALE))
        thymio.set_motor_right_speed(int(y[1]/MOTOR_SCALE))
        print(thymio.get_motor_left_speed())
        print("y0: {} y1: {}".format(int(y[0]/MOTOR_SCALE), int(y[1]/MOTOR_SCALE)))
    else:
        thymio.stop_thymio()
    
    if math.dist([thymio_pose[0], thymio_pose[1]], [obj_pos[0], obj_pos[1]]) < LOC_DIST_THR:
        thymio.stop_thymio()
        return
    time.sleep(0.1)


def angle_two_points(x1, y1, x2, y2):
    return math.atan((y2-y1)/(x2-x1))

# thymio = MyThymio(verbose = True)

# while True:
#     thymio.ser.set_var("motor.left.target", 50)
#     thymio.ser.set_var("motor.right.target", 2**16-10)
#     print(thymio.get_motor_left_speed())
#     print(thymio.get_motor_right_speed())
#     time.sleep(0.5)

# # thymio.set_motor_speeds(50, 50)
# thymio.ser.set_var("motor.left.target", 50)
# time.sleep(4)
# thymio.stop_thymio()    

# th = connect_to_thymio()
# for i in range(4):
#     time.sleep(1)
#     print(i)
# th.set_var("motor.right.target", 50)
# th.set_var("motor.left.target", 50)
# time.sleep(3)
# th.set_var("motor.right.target", 0)
# th.set_var("motor.left.target", 0)