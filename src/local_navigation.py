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
import numpy as np

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Number of proximity sensor values.
NUM_PROX_VALUES = 7

## Number of memories
NUM_MEM = 2

## Number of ground sensors values.
NUM_GND_SENS = 2

## Left weights for local obstacle avoidance, taken from Exercise Session 3
WEIGHTS_LEFT_PROX = [40,  20, -20, -20, -40,  30, -10, 8, 0]

## Right weights for local obstacle avoidance, taken from Exercise Session 3
WEIGHTS_RIGHT_PROX = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

## Left weights for fixed obstacles avoidance.
WEIGHTS_LEFT_GND = [50, -20]

## Right weights for fixed obstacles avoidance.
WEIGHTS_RIGHT_GND = [-20, 50]

## Scale factor for proximity sensors.
PROX_SCALE = 20

## Scale factor for ground sensors.
GND_SCALE = 200

## Scale factor for motors.
MOTOR_SCALE = 15

## Memory scale factor for the motors
MEM_SCALE = 50

## Objective attractiveness coefficient.
OBJ_ATT_COEFF = 4000

## Objective base attractiveness
OBJ_ATT_BASE = 1000

## Local avoidance threshold distance to local objective (in px).
LOC_DIST_THR = 20

## Local avoidance time step
TS_LOCAL = 0.1

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Avoids a local obstacle (not detected by the camera)
#  @param       thymio          Thymio instance.
#  @param       obj_pos         Local objective position (x, y).
#  @param       cam             Camera instance.
#  @param       M               Warp transform matrix.
#  @param       rect_width      Width of rectified image in pixels.
#  @param       rect_height     Height of rectified image in pixels.
def local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height, verbose = False):
    front_prox = np.zeros(NUM_PROX_VALUES + NUM_MEM)
    y = np.zeros(2)
    while True:
        img, img_taken= take_picture(cam)
        # print(img_taken)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width,  rect_height)
            cv2.circle(img_rect, obj_pos, 4, (0, 0, 255), -1)
            thymio_pose, found_thymio = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
            if verbose:
                print("Found thymio: {}".format(found_thymio))
                print(thymio_pose)
            if found_thymio:
                angle = thymio_pose[2]
                cv2.arrowedLine(img_rect, (int(thymio_pose[0]), int(thymio_pose[1])), (int(thymio_pose[0] + math.cos(angle)*50), int(thymio_pose[1] - math.sin(angle)*50)),
                                (128, 0, 255), 3, tipLength = 0.3)
                # Add memory terms
                front_prox[NUM_PROX_VALUES] = y[0]/50
                front_prox[NUM_PROX_VALUES+1] = y[1]/50

                # Local obstacles contribution (repulsive)
                front_prox[0:NUM_PROX_VALUES] = np.divide(thymio.get_prox_horizontal(), PROX_SCALE)

                y = np.zeros(2)
                for i in range(NUM_PROX_VALUES + NUM_MEM):
                    y[0] = y[0] + front_prox[i] * WEIGHTS_LEFT_PROX[i]
                    y[1] = y[1] + front_prox[i] * WEIGHTS_RIGHT_PROX[i]
                
                # Local objective contribution (attractive)
                a_th_obj = angle_two_points(thymio_pose[0], thymio_pose[1], obj_pos[0], obj_pos[1])
                da = thymio_pose[2] - a_th_obj
                if (da < -math.pi):
                    da = da + 2*math.pi
                if (da > math.pi):
                    da = da - 2*math.pi
                if verbose:
                    print("Delta angle: {}".format(da))
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
                if math.dist([thymio_pose[0], thymio_pose[1]], [obj_pos[0], obj_pos[1]]) < LOC_DIST_THR:
                    thymio.stop_thymio()
                    break
            else:
                thymio.stop_thymio()
            cv2.imshow('Local avoidance', img_rect)
            cv2.waitKey(1)
    time.sleep(TS_LOCAL)


## Calculates the angle formed by x-axis and the line intersecting two input points.
#  @param x1    x-coordinate of first point
#  @param y1    y-coordinate of first point
#  @param x2    x-coordinate of second point
#  @param y2    y-coordinate of second point
#  @return      Angle formed by x-axis and the line intersecting two input points.
def angle_two_points(x1, y1, x2, y2):
    return math.atan2(-(y2-y1), x2-x1)