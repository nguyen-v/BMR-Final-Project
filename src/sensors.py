import numpy as np
import math

#import Camera as cmr

def __init__(self):
	self.x_m = 0
	self.y_m = 0
# Returns the sensors' measurements [x_m, y_m, vx_m, vy_m, vr_m, vl_m, theta_m]
def measurements():
#	if cmr.camera_state(): # Function tells you if we can locate the thymio or not.
	cell, x_m, y_m, theta_m = locate_thymio_camera(rectified_image)
	vr_m, vl_m = [th.get_var("motor.right.speed"), th.get_var("motor.left.speed")]
	save_last_position(x_m, y_m)

#	else:
	x_m, y_m = self.get_last_position()
	vx_m = ((vr_m + vl_m) / 2) * math.cos(theta_m)
	vy_m = ((vr_m + vl_m) / 2) * math.sin(theta_m)

	return x_m, y_m, vx_m, vy_m, vr_m, vl_m, theta_m

def set_motor_speed(left_speed, right_speed):
	th.set_var("motor.left.target", left_speed)
	th.set_var("motor.right.target", right_speed)

def get_motor_speed(left_speed, right_speed):
	th.get_var("motor.left.target", left_speed)
	th.get_var("motor.right.target", right_speed)

def set_last_position(self, x_m, y_m):
	self.x_m = x_m
	self.y_m = y_m
	
def get_last_position(self):
	return self.x_m, self.y_m