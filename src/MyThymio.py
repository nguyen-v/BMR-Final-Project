## 
# @file MyThymio.py
#
# @brief Definition MyThymio class.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import time
import math
import numpy as np

## Custom modules
from thymio_connection import connect_to_thymio
from locate_thymio_goal import locate_thymio_camera
from create_map import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Maximum motor speed (raw value).
MAX_RAW_MOTOR_SPEED = 2**16

## Maximum motor speed (converted to negative and positive range).
MAX_MOTOR_SPEED = 500

## Base motor speed.
BASE_SPEED = 100
# BASE_SPEED = 150

## Rotation coefficient.
ROT_COEFF = 1.5   # for BASE_SPEED = 100
# ROT_COEFF = 0.942   # for BASE_SPEED = 150

## Speed conversion factor (raw values to px/s)
SPEED_COEFF = 0.218

## Number of proximity sensor values.
NUM_PROX_VALUES = 7

# ========================================================================== #
#  Classes.                                                                  # 
# ========================================================================== #

class MyThymio():

    def __init__(self, verbose = False):
        self.ser = connect_to_thymio(verbose = verbose)
        self.last_theta_m = 0
        self.last_thymio_pos = np.zeros(2)

    ## Return position and velocity measurements
    #  @param   img_rect    Rectified image.
    #  @return  x_meas      Measured Thymio x coordinate.
    #  @return  y_meas      Measured Thymio y coordinate.
    #  @return  vx_meas     Measured Thymio x velocity.
    #  @return  vy_meas     Measured Thymio y velocity.
    #  @return  obstructed  True if camera is obstructed.
    def get_measurements(self, img_rect):
            obstructed = False
            thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))

            speed = (self.get_motor_left_speed() + self.get_motor_right_speed())/2 * SPEED_COEFF
            x_meas = 0
            y_meas = 0
            vx_meas = 0
            vy_meas = 0
            # If Thymio found, update x, y, vx and vy measured
            if thymio_found:
                self.set_last_angle(thymio_pos[2])
                x_meas = thymio_pos[0]
                y_meas = thymio_pos[1]

                cv2.circle(img_rect, [int(x_meas), int(y_meas)] , 4, (0, 0, 255), -1)

                vx_meas = speed * math.cos(self.get_last_angle())
                vy_meas = -speed * math.sin(self.get_last_angle())

            else:
                obstructed = True
            return x_meas, y_meas, vx_meas, vy_meas, obstructed

    
    ## Set motor right speed (positive or negative).
    #  @param       speed       Positive or negative speed value.
    def set_motor_right_speed(self, speed):
        try:
            if speed >= 0:
                self.ser.set_var("motor.right.target", speed)
            else:
                self.ser.set_var("motor.right.target", MAX_RAW_MOTOR_SPEED + speed)
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            pass

    ## Get motor right speed (positive or negative).
    #  @return      speed       Positive or negative right speed value.
    def get_motor_right_speed(self):
        try:
            speed = self.ser.get_var("motor.right.speed")
            if speed <= MAX_MOTOR_SPEED:
                return speed
            else:
                return speed - MAX_RAW_MOTOR_SPEED
        except (IndexError, KeyError):
            return 0
            
    ## Set motor left speed (positive or negative).
    #  @param       speed       Positive or negative speed value.
    def set_motor_left_speed(self, speed):
        try:
            if speed >= 0:
                self.ser.set_var("motor.left.target", speed)
            else:
                self.ser.set_var("motor.left.target", MAX_RAW_MOTOR_SPEED + speed)
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            pass

    ## Get motor left speed (positive or negative).
    #  @return      speed       Positive or negative left speed value.
    def get_motor_left_speed(self):
        try:
            speed = self.ser.get_var("motor.left.speed")
            if speed <= MAX_MOTOR_SPEED:
                return speed
            else:
                return speed - MAX_RAW_MOTOR_SPEED
        except (IndexError, KeyError):
            return 0

    ## Set motor speeds (positive or negative).
    #  @param       speed_l     Positive or negative left speed value. 
    #  @param       speed_r     Positive or negative right speed value.     
    def set_motor_speeds(self, speed_l, speed_r):
        self.set_motor_left_speed(speed_l)
        self.set_motor_right_speed(speed_r)

    ## Stop Thymio's motors.
    def stop_thymio(self):
        self.set_motor_speeds(0, 0)

    ## Rotate Thymio by given angle.
    #  @param       angle       Angle in radian. Positive angle is clockwise.
    def rotate_thymio(self, angle):
        if angle >= 0:
            self.set_motor_speeds(BASE_SPEED, -BASE_SPEED)
        else:
            self.set_motor_speeds(-BASE_SPEED, BASE_SPEED)
        time.sleep(abs(angle)*ROT_COEFF)
        self.stop_thymio()

        final_angle = self.get_last_angle() - angle
        if final_angle < -math.pi:
            final_angle = final_angle + 2*math.pi
        elif final_angle > math.pi:
            final_angle = final_angle - 2*math.pi

        self.set_last_angle(final_angle)

    ## Saves the last position, and orientation of the Thymio .
	#  @param       thymio_pos          The last position of the Thymio (xy coordinates).
    def set_last_position(self, thymio_pos):
        self.last_thymio_pos = thymio_pos

	## Returns the last position and orientation of the Thymio .
	#  @return      thymio_pos          The last position of the Thymio (x, y, theta).
    def get_last_position(self):
        thymio_pos = np.append(self.last_thymio_pos, self.last_theta_m)
        return thymio_pos

    ## Returns raw horizontal proximity sensor values.
    #  @return      prox_horizontal     Array of proximity sensor raw values.
    def get_prox_horizontal(self):
        try:
            return self.ser["prox.horizontal"]
        except (KeyError, ValueError):
            return np.zeros(1, NUM_PROX_VALUES)
    
    ## Saves the last angle of the Thymio.
	#  @param       theta_m             The last position of the thymio (xy coordinates).   
    def get_last_angle(self):
        return self.last_theta_m

    ## Returns last angle of the Thymio.
	#  @return      theta_m             The last position of the thymio (xy coordinates).  
    def set_last_angle(self, theta_m):
        self.last_theta_m = theta_m

