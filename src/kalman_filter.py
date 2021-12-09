## 
# @file kalman.py
#
# @brief Definition of functions to estimate system states with a Kalman filter.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import numpy as np
import math

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

# Sampling time in seconds
T_s = 0.1

## Kalman filter matrices
A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0, 0, 0, 1.0]])
B = np.array([[T_s, 0], [0, T_s], [1.0, 0], [0, 1.0]])
Q = np.diag([5, 5, 10, 10])

# ========================================================================== #
#  Exported functions                                                        # 
# ========================================================================== #

def kalman_filter(x_meas, y_meas, vx_meas, vy_meas, x_est_prev, P_est_prev, dvx = 0, dvy = 0, obstructed = False):
    
    ## Prediciton through the a priori estimate
    # estimated mean of the state
    U_in = np.array([dvx, dvy])
    x_est_a_priori = A @ x_est_prev + B @ U_in
    
    # Estimated covariance of the state
    P_est_a_priori = A @ (P_est_prev @ A.T)
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
    
    ## Update         
    if obstructed:
        R = np.diag([math.inf, math.inf, math.inf, math.inf])
    else:
        R = np.diag([1, 1, 5, 5])
    y = np.array([x_meas, y_meas, vx_meas, vy_meas])
    H = np.eye(4)

    # innovation / measurement residual
    i = y - H @ x_est_a_priori

    # measurement prediction covariance
    S = H @ (P_est_a_priori @ H.T) + R
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = P_est_a_priori @ (H.T @ np.linalg.inv(S))


    # a posteriori estimate
    x_est = x_est_a_priori + K @ i
    P_est = P_est_a_priori - K @ (H @ P_est_a_priori)

    return x_est, P_est
