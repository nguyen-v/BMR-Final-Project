## 
# @file kalman.py
#
# @brief Definition of class and functions to estimate system states with a Kalman filter.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import numpy as np
import math
import MyThymio

from numpy.linalg import inv

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #


# Input states of kalman_filter (Absolute Position (x,y), Absolute Speed(vx,vy), Motor speed(vr,vl) 
# 								 and Angle(theta))
# The inputs are defined by the Absolute speeds (u_vx, u_vy) and motor speeds (u_vx, u_vy).

# ========================================================================== #
#  Classes.                                                       			 # 
# ========================================================================== #

class kalman_filter():

	def __init__(self):

		# Sampling time in seconds
		self.T_s = 0.1
		
		#Values made only for testing, to be deleted.
		self.x_test = 0.0
		self.y_test = 0.0
		self.vx_test = 0.5
		self.vy_test = 0.5
		self.vl_test = 0.5
		self.vr_test = 0.5
		self.angle_test = np.deg2rad(20.0)
		self.K_t = 0.0

		self.X_pred = np.zeros((4,1))
		self.X_prev = np.zeros((4,1))
		self.U_prev = np.zeros((2,1))
		self.P_prev = np.zeros((4,4))
		self.R_prev = 0	


		self.Q = np.diag([0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined
		self.R = np.diag([0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined			 

	## Establishes the system's observation model.
	#  @return X_pred   The à-prioris estimation of the states.
	#  @return A  		Matrix describing the system's evolution.
	#  @return B        Matrix linking the system's states and inputs.
	def observation_model(self):

		A = np.array([[1.0, 0, 0, 0],
					[0, 1.0, 0, 0],
					[0, 0, 1.0, 0],
					[0, 0, 0, 1.0]])


		B = np.array([[self.T_s, 0],
					[0, self.T_s],
					[1.0, 0],
					[0, 1.0]])
		
		# Noise vector generation :
		W = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0]) @ np.random.randn(4,1)
		print(self.U_prev)
		X_pred = A @ self.X_prev + B @ self.U_prev + W

		return X_pred, A, B

	## Establishes the system's measurement model.
	#  @return Y		The measurement model of the system.
	#  @return C  		Matrix describing the system's measurements.
	def measurement_model(self, thymio_pos, abs_v):

		C = np.identity(4)

		# Noise vector generation :
		V = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0]) @ np.random.randn(4,1)

		Y = np.array([[thymio_pos[0]], [thymio_pos[1]], [abs_v[0]], [abs_v[1]]]) + V
		#Y = sens.measurements() + V

		return Y, C

	## Filters the noise of our system's states.
	#  @return X_post		The à-posteriori estimates of the system's states.
	def filter(self, thymio_pos, abs_v, new_T_s):
		
		update_sampling_time(new_T_s)
		X_pred, A, B = self.observation_model()
		self.kalman_save_prediction(X_pred)

		#   Values put for testing
		Y, C =self.measurement_model(thymio_pos, abs_v)
		
		P_pred = A @ self.P_prev @ A.transpose() + self.Q

		# Innovation calculation : Difference between the measure and the prediction
		I = Y - C @ X_pred

		# Innovation variance calculation :
		S_post = C @ P_pred @ C.transpose() + self.R

		# Optimal gain for correction :
		self.K_t= P_pred @ C.transpose() @ inv(S_post) 

		# Calculation of the posteriori states :
		X_post = X_pred + self.K_t@ I

		# Calculation of posteriori state covariance matrix :
		P_post = (I - self.K_t@ C) @ P_pred

		self.kalman_save_post_states(X_post, P_post, S_post)

		return X_post

	## Saves the last calculated à-prioris state estimations
	#  @param X_pred		The à-prioris estimation of the states.
	def kalman_save_prediction(self, X_pred):
		self.X_pred = X_pred
	
	## Returns the last calculated à-prioris state estimations
	#  @return X_pred		The à-prioris estimation of the states.
	def kalman_get_prediction(self):
		return self.X_pred

	## Saves the last calculated à-postériori state estimations and covariance matrices
	#  @param X_post		The à-prioris estimation of the states.
	#  @param P_post		The à-prioris estimation of the states.
	#  @param S_post		The à-prioris estimation of the states.
	def kalman_save_post_states(self, X_post, P_post, S_post):
		self.X_prev, self.P_prev, self.R_prev = X_post, P_post, S_post

	## Returns the last calculated à-postériori state estimations and covariance matrices
	#  @return X_post		The à-prioris estimation of the states.
	#  @return P_post		The à-prioris estimation of the states.
	#  @return S_post		The à-prioris estimation of the states.
	def kalman_get_prev_states(self):
		return self.X_prev, self.U_prev, self.P_prev, self.R_prev

	# Only made for testing, to be deleted
	def measurements_test(self, vl_test, vr_test):
		self.vl_test = vl_test
		self.vr_test = vr_test
		self.angle_test = ((self.angle_test + (vr_test-vl_test)*0.5*self.T_s) % (2*math.pi))
		self.vx_test = (self.vr_test+self.vl_test)*math.cos(self.angle_test)/2
		self.vy_test = (self.vr_test+self.vl_test)*math.sin(self.angle_test)/2
		self.x_test = self.x_test + self.T_s*self.vx_test
		self.y_test = self.y_test + self.T_s*self.vy_test

	# Only made for testing, to be deleted
	def return_meas_test(self):
		return np.array([[self.x_test], [self.y_test]])

	## Updates the inputs of the system into the Kalman filter
	#  @param U_in		The new inputs of the system
	def save_input_control(self, U_in):
		self.U_prev = U_in
	
	def update_sampling_time(self, T_s):
		self.T_s = T_s
		

# # Input states of extended_kalman_filter (Absolute Position (x,y), Absolute Speed(vx,vy), Angle(theta))
# class extended_kalman():

# 	def __init__(self):

# 		T_s = 0.1 #ms

# 		self.Q = np.diag([0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined
# 		self.R = np.diag([0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined		

# 		self.X_pred = np.zeros((4,1))
# 		self.X_prev = np.zeros((4,1))
# 		self.Y_prev = np.zeros((4,1))
# 		self.U_prev = np.array([0.1])
# 		self.P_prev = np.zeros((4,4))
# 		self.R_prev = 0	

# 		#Values made only for testing, to be deleted.
# 		self.x_test = 0.0
# 		self.y_test = 0.0
# 		self.v_test = 0.5
# 		self.vx_test = 0.5
# 		self.vy_test = 0.5
# 		self.vl_test = 0.2
# 		self.vr_test = 0.2
# 		self.angle_test = np.deg2rad(20.0)

# 	def filter(self):

# 		self.measurements_test(0.5,0.5)
# 		A = np.array([[1.0, 0, T_s, 0, 0],
# 					  [0, 1.0, 0, T_s, 0],
# 					  [0, 0, 1.0, 0, 0],
# 					  [0, 0, 0, 1.0, 0],
# 					  [0, 0, 0, 0, 1.0]])

# 		B = np.array([[T_s, T_s*math.cos(self.X_prev[3,0]/2) ],
# 					  [T_s * math.sin(self.X_prev[3,0])],
# 					  [1.0],
# 				      [0]])

# 		C = np.identity(4)

# 		# Noise vector generation :
# 		W = 1e-3*np.diag([1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(4,1)
# 		V = 1e-3*np.diag([1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(4,1)
# 		# Covariance for KF simulation

# 		self.X_pred = A @ self.X_prev + B @ self.U_prev + W
# 		self.kalman_save_prediction(self.X_pred)

# 		J_pred = np.array([[1.0, 0, T_s*math.cos(self.X_pred[3,0]), -T_s*self.X_pred[2,0]*math.sin(self.X_pred[3,0])],
# 						   [0, 1.0, T_s*math.sin(self.X_pred[3,0]), T_s*self.X_pred[2,0]*math.cos(self.X_pred[3,0])],
# 						   [0, 0, 1.0, 0],
# 						   [0, 0, 0, 1.0]]),		
		
# 		Y = np.array([[self.x_test], [self.y_test], [self.v_test], [self.angle_test]]) + V
# 		self.kalman_save_prev_measurements(Y)
# 		print(J_pred)
# 		J_m = np.identity(4)

# 		P_pred = J_pred @ self.P_prev @ J_pred.transpose() + self.R

# 		# Innovation calculation : Difference between the measure and the prediction
# 		I = Y - C @ self.X_pred
# 		K = P_pred @ J_m.transpose() @ inv(J_m @ P_pred @ J_m.transpose() + self.Q)

# 		X_post = self.X_pred + K @ (Y - J_m @ self.X_pred) 
# 		P_post = (I - K @ J_m) @ P_pred

# 		print(X_post)
# 		self.kalman_save_post_states(X_post, P_post)

# 		return X_post

# 	def kalman_save_prediction(self, X_pred):
# 		self.X_pred = X_pred
	
# 	def kalman_get_prediction(self):
# 		return self.X_pred

# 	def kalman_save_post_states(self, X_post, P_post):
# 		self.X_prev, self.P_prev = X_post, P_post

# 	def kalman_get_prev_states(self):
# 		return self.X_prev

# 	def kalman_save_prev_measurements(self, Y_prev):
# 		self.Y_prev = Y_prev

# 	def kalman_set_prev_measurements(self):
# 		return self.Y_prev

# 	# Only made for testing, to be deleted
# 	def measurements_test(self, vl_test, vr_test):
# 		self.vl_test = vl_test
# 		self.vr_test = vr_test
# 		self.angle_test = ((self.angle_test + (vr_test-vl_test)*0.5*T_s) % (2*math.pi))
# 		self.vx_test = (self.vr_test+self.vl_test)*math.cos(self.angle_test)/2
# 		self.vy_test = (self.vr_test+self.vl_test)*math.sin(self.angle_test)/2
# 		self.v_test = (self.vr_test+self.vl_test)/2
# 		self.yaw = self.v_test / (0.7*(vl_test-vr_test)/(vl_test - vr_test))
# 		self.x_test = self.x_test + T_s*self.vx_test
# 		self.y_test = self.y_test + T_s*self.vy_test

# 	def return_meas_test(self):
# 		return np.array([[self.x_test], [self.y_test]])

	