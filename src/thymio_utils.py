## 
# @file thymio_connection.py
#
# @brief Definition of functions to set or read data from Thymio.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import time

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
ROT_COEFF = 1.362   # for BASE_SPEED = 100
# ROT_COEFF = 0.942   # for BASE_SPEED = 150

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Set motor right speed (positive or negative).
#  @param       th          Thymio serial connection instance.
#  @param       speed       Positive or negative speed value.
def set_motor_right_speed(th, speed):
    try:
        if speed >= 0:
            th.set_var("motor.right.target", speed)
        else:
            th.set_var("motor.right.target", MAX_RAW_MOTOR_SPEED + speed)
    except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
        pass

## Get motor right speed (positive or negative).
#  @param       th          Thymio serial connection instance.
#  @return      speed       Positive or negative right speed value.
def get_motor_right_speed(th):
    try:
        speed = th.get_var("motor.right.speed")
        if speed <= MAX_MOTOR_SPEED:
            return speed
        else:
            return speed - MAX_RAW_MOTOR_SPEED
    except (IndexError, KeyError):
        return 0
        

## Set motor left speed (positive or negative).
#  @param       th          Thymio serial connection instance.
#  @param       speed       Positive or negative speed value.
def set_motor_left_speed(th, speed):
    try:
        if speed >= 0:
            th.set_var("motor.left.target", speed)
        else:
            th.set_var("motor.left.target", MAX_RAW_MOTOR_SPEED + speed)
    except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
        pass

## Get motor left speed (positive or negative).
#  @param       th          Thymio serial connection instance.
#  @return      speed       Positive or negative left speed value.
def get_motor_left_speed(th):
    try:
        speed = th.get_var("motor.left.speed")
        if speed <= MAX_MOTOR_SPEED:
            return speed
        else:
            return speed - MAX_RAW_MOTOR_SPEED
    except (IndexError, KeyError):
        return 0

## Set motor speeds (positive or negative).
#  @param       th          Thymio serial connection instance.
#  @param       speed_l     Positive or negative left speed value. 
#  @param       speed_r     Positive or negative right speed value.     
def set_motor_speeds(th, speed_l, speed_r):
    set_motor_left_speed(th, speed_l)
    set_motor_right_speed(th, speed_r)

## Stop Thymio's motors.
#  @param       th          Thymio serial connection instance.
def stop_thymio(th):
    set_motor_speeds(th, 0, 0)

## Rotate Thymio by given angle.
#  @param       th          Thymio serial connection instance.
#  @param       angle       Angle in radian. Positive angle is counter-clockwise.
def rotate_thymio(th, angle):
    if angle >= 0:
        set_motor_speeds(th, -BASE_SPEED, BASE_SPEED)
    else:
        set_motor_speeds(th, BASE_SPEED, -BASE_SPEED)
    time.sleep(abs(angle)*ROT_COEFF)
    stop_thymio(th)

