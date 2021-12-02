# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

from thymio_connection import connect_to_thymio
from locate_thymio_goal import locate_thymio_camera
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

    
# ## Returns the position, absolute speeds, motor speeds and orientation of the thymio .
# 	#  @return thymio_pos   The position of the thymio.
# 	#  @return abs_v 		The speeds of the thymio relative to the x and y axis.
# 	#  @return v_m      	The speeds of the thymio's wheels
# 	#  @return theta_m		The thymio's current orientation
#     def measurements(camera_state, rect_map):
#         thymio_pos, found_thymio = locate_thymio_camera(rect_map, "cartesian", [])
#         v_m = numpy.array([self.get_motor_left_speed, self.get_motor_right_speed()])
#         abs_v = numpy.array([((vr_m + vl_m) / 2) * math.cos(theta_m),((vr_m + vl_m) / 2) * math.sin(theta_m)])  
#         if found_thymio: # Function tells you if we can locate the thymio or not.
#             self.save_last_position(thymio_pos, theta_m)        
#         else:
#             thymio_pos, theta_m = self.get_last_position()     
#         return thymio_pos, theta_m, abs_v, v_m
    
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

    ## Saves the last position, and orientation of the thymio .
	#  @param thymio_pos  The last position of the thymio.
	#  @param theta_m     The last orientation of the thymio.
    def set_last_position(self, thymio_pos, theta_m):
        self.last_thymio_pos = np.array([thymio_pos[0],thymio_pos[1]])
        self.last_theta_m = theta_m

	## Returns the last position and orientation of the thymio .
	#  @return thymio_pos  The last position of the thymio.
	#  @return theta_m     The last orientation of the thymio.
    def get_last_position(self):
        return self.last_thymio_pos, self.last_theta_m

    ## Returns raw horizontal proximity sensor values.
    #  @return prox_horizontal      Array of proximity sensor raw values.
    def get_prox_horizontal(self):
        try:
            return self.ser["prox.horizontal"]
        except (KeyError, ValueError):
            pass
    
    def get_gnd_sensors(self):
        try:
            return self.ser["prox.ground.reflected"]
        except (KeyError, ValueError):
            pass


