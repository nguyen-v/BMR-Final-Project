import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import math

import sensors as sens

# Input states of kalman_filter (Absolute Position (x,y), Absolute Speed(vx,vy), Motor speed(vr,vl) 
# 								 and Angle(theta))
# The inputs are defined by the Absolute speeds (u_vx, u_vy) and motor speeds (u_vx, u_vy).
class kalman_filter():

	def __init__(self):
		self.T_s = 0.1 # sampling time
		self.A = np.array([[1.0, 0, self.T_s, 0, 0, 0, 0],
					[0, 1.0, 0, self.T_s, 0, 0, 0],
					[0, 0, 1.0, 0, 0, 0, 0],
					[0, 0, 0, 1.0, 0, 0, 0],
					[0, 0, 0, 0, 1.0, 0, 0],
					[0, 0, 0, 0, 0, 1.0, 0],
					[0, 0, 0, 0, -0.5*self.T_s, 0.5*self.T_s, 1.0]])	

		self.B = np.array([[self.T_s, 0, 0, 0],
					[0, self.T_s, 0, 0],
					[1.0, 0, 0, 0],
					[0, 1.0, 0, 0],
					[0, 0, 1.0, 0],
					[0, 0, 0, 1.0],
					[0, 0, -0.5*self.T_s,  0.5*self.T_s]])

		self.C = np.identity(7)

		#Values made only for testing, to be deleted.
		self.x_test = 0.0
		self.y_test = 0.0
		self.vx_test = 0.5
		self.vy_test = 0.5
		self.vl_test = 0.5
		self.vr_test = 0.5
		self.angle_test = np.deg2rad(20.0)

		self.X_pred = np.zeros((7,1))
		self.X_prev = np.zeros((7,1))
		self.U_prev = np.zeros((4,1))
		self.P_prev = np.zeros((np.size(self.A,0),np.size(self.A,0)))
		self.R_prev = 0	


		self.Q = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined
		self.R = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined			 




	def filter(self):

		# Noise vector generation :
		W = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(7,1)
		V = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(7,1)
		# Covariance for KF simulation

		X_pred = self.A @ self.X_prev + self.B @ self.U_prev + W
		
		self.kalman_save_prediction(X_pred)

	#	Y = sens.measurements() + V

	#   Values put for testing
		self.measurements_test()
		Y = np.array([[self.x_test], [self.y_test], [self.vx_test], [self.vy_test], [self.vr_test], [self.vl_test], [self.angle_test]]) + V
		
		P_pred = self.A @ self.P_prev @ self.A.transpose() + self.Q

		# Innovation calculation : Difference between the measure and the prediction
		I = Y - self.C @ X_pred
		

		# Innovation variance calculation :
		S_post = self.C @ P_pred @ self.C.transpose() + self.R

		# Optimal gain fore correction :
		K_t = P_pred @ self.C.transpose() @ inv(S_post) 
		# Calculation of the posteriori states :
		X_post = X_pred + K_t @ I

		# Calculation of posteriori state covariance matrix :
		P_post = (I - K_t @ self.C) @ P_pred
		self.kalman_save_post_states(X_post, P_post, S_post)

		return X_post, P_post, S_post

	def kalman_save_prediction(self, X_pred):
		self.X_pred = X_pred
	
	def kalman_get_prediction(self):
		return self.X_pred

	def kalman_save_post_states(self, X_post, P_post, S_post):
		self.X_prev, self.P_prev, self.R_prev = X_post, P_post, S_post

	def kalman_get_prev_states(self):
		return self.X_prev, self.U_prev, self.P_prev, self.R_prev

	# Only made for testing, to be deleted
	def measurements_test(self):
		self.angle_test = (self.angle_test + np.deg2rad(1))
		self.vl_test = 0.5
		self.vr_test = 0.5
		self.vx_test = (self.vr_test+self.vl_test)*math.cos(self.angle_test)/2
		self.vy_test = (self.vr_test+self.vl_test)*math.sin(self.angle_test)/2
		self.x_test = self.x_test + self.T_s*self.vx_test
		self.y_test = self.y_test + self.T_s*self.vy_test

	def return_meas_test(self):
		return np.array([[self.x_test], [self.y_test]])
		

# Input states of extended_kalman_filter (Absolute Position (x,y), Absolute Speed(vx,vy), Angle(theta))
class extended_kalman():

	def __init__(self):

		self.T_s = 0.1 #ms
		self.A = np.array([[1.0, 0, T_s, 0, 0],
				[0, 1.0, 0, T_s, 0],
				[0, 0, 1.0, 0, 0],
				[0, 0, 0, 1.0, 0],
				[0, 0, 0, 0, 1.0]])

		self.B = np.array([[T_s * math.cos(theta_m), 0],
					[0, T_s * math.sin(theta_m)],
					[1.0, 0, 0, 0],
					[0, 1.0, 0, 0],
					[0, 0, 2.0, 2.0]])

		self.Q = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined
		self.R = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])		# Temporary, to be determined		

		self.X_pred = np.zeros((5,1))
		self.X_prev = np.zeros((5,1))
		self.Y_prev = np.zeros((5,1))
		self.U_prev = np.zeros((4,1))
		self.P_prev = np.zeros((np.size(self.A,0),np.size(self.A,0)))
		self.R_prev = 0	

		#Values made only for testing, to be deleted.
		self.x_test = 0.0
		self.y_test = 0.0
		self.vx_test = 0.5
		self.vy_test = 0.5
		self.angle_test = np.deg2rad(20.0)

	def extended_kalman_filter(self):

		# Noise vector generation :
		W = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(5,1)
		V = 1e-2*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, np.deg2rad(20.0)]) @ np.random.randn(5,1)
		# Covariance for KF simulation

		self.X_pred = self.A @ self.X_prev + self.B @ self.U_prev + W
		J_pred = np.array([[1.0, 0, self.T_s, 0, -self.T_s*X_pred[3,:]*math.sin(X_pred[5,:])],
						   [0, 1.0, 0, self.T_s, self.T_s*X_pred[4,:]*math.cos(X_pred[5,:])],
						   [0, 0, 1, 0, 0],
						   [0, 0, 0, 1, 0],
						   [0, 0, 0, 0, 1]])		

		Y = np.array([[self.x_test], [self.y_test], [self.vx_test], [self.vy_test], [self.angle_test]]) + V
		J_m = np.identity(5)

		P_pred = J_pred @ self.P_prev @ J_pred.transpose() + self.R

		# Innovation calculation : Difference between the measure and the prediction
		I = Y - self.C @ X_pred
		
		K = P_pred @ J_m.transpose() @ inv(J_m @ P_pred @ J_m.transpose() + self.Q)

		X_post = X_pred + K @ (Y - Y_prev)
		P_post = (I - K @ J_m) @ P_pred

		return X_post


	