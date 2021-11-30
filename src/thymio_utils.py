## 
# @file thymio_utils.py
#
# @brief Definition of class and functions to manipulate Thymio

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import numpy as np
import math
import time
import Thymio as th

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

SPEED_CONVERSION = 0.33333 # to be determined

FULL_ROTATION_TIME = 0.55555 # to be determined

BASE_RIGHT_SPEED = 0.5 # To be determined

BASE_LEFT_SPEED = 0.5 # To be determined

NOMINAL_BASE_SPEED = 0.5 # To be determined

STOP_MOTOR = 0

# ========================================================================== #
#  Classes.                                                       			 # 
# ========================================================================== #

class thymio_control():

	def __init__(self):
		self.last_thymio_pos = np.zeros(2,1)
		self.last_theta_m = 0
		self.th_mode = th.RemoteNode()

	## Returns the position, absolute speeds, motor speeds and orientation of the thymio .
	#  @return thymio_pos   The position of the thymio.
	#  @return abs_v 		The speeds of the thymio relative to the x and y axis.
	#  @return v_m      	The speeds of the thymio's wheels
	#  @return theta_m		The thymio's current orientation
	def measurements():
#		if cmr.camera_state(): # Function tells you if we can locate the thymio or not.
			cell, thymio_pos, theta_m = locate_thymio_camera(rectified_image) # Necessitates the appropriate function...
			v_m = numpy.array([self.th_mode.get_var("motor.right.speed"), self.th_mode.get_var("motor.left.speed")])
			save_last_position(thymio_pos, theta_m)

#		else:
			thymio_pos, theta_m = self.get_last_position()
			v_m = numpy.array([self.th_mode.get_var("motor.right.speed"), self.th_mode.get_var("motor.left.speed")])
			abs_v = numpy.array([((vr_m + vl_m) / 2) * math.cos(theta_m),((vr_m + vl_m) / 2) * math.sin(theta_m)])
			
#		return thymio_pos, abs_v, v_m, theta_m
# 	
	## Rotates the thymio towards a given angle and following a specific direction
	#  @param angle_rad   The next orientation of the thymio .
	#  @param direction   The direction of turning.
	def thymio_rotate(self, angle_rad, direction):
		self.set_left_motor_speed(BASE_LEFT_SPEED*direction)
		self.set_right_motor_speed(-BASE_RIGHT_SPEED*direction)

		time.sleep(FULL_ROTATION_TIME*angle_rad/(2*math.pi))

		self.set_left_motor_speed(STOP_MOTOR)
		self.set_right_motor_speed(STOP_MOTOR)

	## Saves the last position, and orientation of the thymio .
	#  @param thymio_pos  The last position of the thymio.
	#  @param theta_m     The last orientation of the thymio.
	def set_last_position(self, thymio_pos, theta_m):
		self.last_thymio_pos = np.array([thymio_pos[0],thymio_pos[1]])
		self.last_theta_m = theta_m

	## Sets the left motor's target speed.
	#  @param left_speed   The left motor's target speed.
	def set_left_motor_speed(left_speed):
		self.th_mode.set_var("motor.left.target", left_speed)

	## Sets the right motor's target speed.
	#  @param right_speed   The right motor's target speed.
	def set_right_motor_speed(right_speed):
		self.th_mode.set_var("motor.right.target", right_speed)

	## returns the left motor's target speed.
	#  @return left_speed   The left motor's target speed.
	def get_left_motor_speed(lef):
		return self.th_mode.get_var("motor.left.speed")

	## Sets the right motor's target speed.
	#  @return right_speed   The right motor's target speed.
	def get_right_motor_speed(right_speed):
		return self.th_mode.get_var("motor.right.speed")

	## Converts a measured speed to meter per seconds
	#  @param speed   A speed in the thymio's units.
	def speed2ms(speed):
		speed *= self.speed_conversion
		return speed

	## Converts a speed to thymio's units
	#  @param speed_ms		A speed in meter per seconds
	def ms2speed(speed_ms):
		speed_ms /= self.speed_conversion
		return speed_ms
		
	## Returns the thymio's last position.
	def get_last_position(self):
		return self.last_thymio_pos

	def ground_measure(self):
		return self.th_mode.get_var("prox.ground.delta")
