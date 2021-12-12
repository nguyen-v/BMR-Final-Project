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
Q = np.diag([2, 2, 3, 3])
H = np.eye(4)

# ========================================================================== #
#  Exported functions                                                        # 
# ========================================================================== #

## Returns new a posteriori estimates from measurement and previous a posteriori estimates
#  @param   x_meas          Measured x-coordinate.
#  @param   y_meas          Measured y-coordinate.
#  @param   vx_meas         Measured x-velocity.
#  @param   vy_meas         Measured y-velocity.
#  @param   dvx             y-velocity control input.
#  @param   dvy             x-velocity control input.
#  @param   obstructed      Set to true if measurements cannot be taken (camera is obstructed).
#  @return  x_est           A posteriori estimate.
#  @return  P_est           A posteriori covariance matrix.
#  @note                    Adapted from Kalman filter of solutions of exercise session 7.
def kalman_filter(x_meas, y_meas, vx_meas, vy_meas, x_est_prev, P_est_prev, dvx = 0, dvy = 0, obstructed = False):
    
    # The states are:
    # x_est = [x, y, vx, vy]

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
        R = np.diag([0.25, 0.25, 0.30, 0.30])

    y = np.array([x_meas, y_meas, vx_meas, vy_meas])

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

## Updates the sampling time for Kalman filter
#  @param   new_Ts      New sampling time in seconds.
#  @note                This function is necessary when the execution time is significantly slower
#                       than the sampling time (happens when we were recording, which slows down the
#                       computer).
def update_sampling_time(new_Ts):
    global A
    A = np.array([[1.0, 0, new_Ts, 0],[0, 1.0, 0, new_Ts],[0, 0, 1.0, 0],[0, 0, 0, 1.0]])