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
MAX_MOTOR_SPEED = 2**16

## Time interval between each try to set/read variable.
DELTA_T_VAR = 0.1

## Base motor speed.
BASE_SPEED = 100

## Rotation coefficient.
ROT_COEFF = 200/BASE_SPEED

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Set motor right speed (raw value).
#  @param       th      Thymio serial connection instance.
#  @param       speed   Raw speed value.
def set_motor_right_speed(th, speed):
    speed_set = False
    while not speed_set:
        try:
            if speed >= 0:
                th.set_var("motor.right.target", speed)
            else:
                th.set_var("motor.right.target", MAX_MOTOR_SPEED + speed)
            speed_set = True
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            time.sleep(DELTA_T_VAR)
            pass


## Set motor left speed (raw value).
#  @param       th          Thymio serial connection instance.
#  @param       speed       Raw speed value.
def set_motor_left_speed(th, speed):
    speed_set = False
    while not speed_set:
        try:
            if speed >= 0:
                th.set_var("motor.left.target", speed)
            else:
                th.set_var("motor.left.target", MAX_MOTOR_SPEED + speed)
            speed_set = True
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            time.sleep(DELTA_T_VAR)
            pass

## Set motor speeds (raw values).
#  @param       th          Thymio serial connection instance.
#  @param       speed_l     Raw left speed value. 
#  @param       speed_r     Raw right speed value.     
def set_motor_speeds(th, speed_l, speed_r):
    set_motor_left_speed(th, speed_l)
    set_motor_right_speed(th, speed_r)

## Stop Thymio's motors.
#  @param       th          Thymio serial connection instance.
def stop_thymio(th):
    set_motor_speeds(th, 0, 0)

## Rotate Thymio
#  @param       th          Thymio serial connection instance.
#  @param       angle       Angle in radian. Positive angle is counter-clockwise.
def rotate_thymio(th, angle):
    if angle >= 0:
        set_motor_speeds(th, -BASE_SPEED, BASE_SPEED)
    else:
        set_motor_speeds(th, BASE_SPEED, -BASE_SPEED)
    time.sleep(abs(angle)*ROT_COEFF)
    stop_thymio(th)

