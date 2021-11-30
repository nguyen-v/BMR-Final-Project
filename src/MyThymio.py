# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

from thymio_connection import connect_to_thymio
import time
import numpy as np

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
#  Classes.                                                                  # 
# ========================================================================== #

class MyThymio():

    def __init__(self, verbose = False):
        self.ser = connect_to_thymio(verbose = verbose)
        self.last_theta_m = 0
        self.last_thymio_pos = np.zeros(2)

    
    #def measurements():
    
    
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
    def get_motor_left_speed(self, speed):
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


    ## Saves the last position, and orientation of the thymio .
	#  @param thymio_pos  The last position of the thymio.
	#  @param theta_m     The last orientation of the thymio.
    def set_last_position(self, thymio_pos, theta_m):
        self.last_thymio_pos = np.array([thymio_pos[0],thymio_pos[1]])
        self.last_theta_m = theta_m

	## Saves the last position, and orientation of the thymio .
	#  @param thymio_pos  The last position of the thymio.
	#  @param theta_m     The last orientation of the thymio.
    def set_last_position(self, thymio_pos, theta_m):
        self.last_thymio_pos = np.array([thymio_pos[0],thymio_pos[1]])
        self.last_theta_m = theta_m

