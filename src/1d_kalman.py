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
from camera import *
from locate_thymio_goal import *
from create_map import *
import matplotlib.pyplot as plt
from MyThymio import *

from numpy.linalg import inv

T_s = 0.1

A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0,0,0,1.0]])
B = np.array([[T_s, 0], [0, T_s], [1.0, 0], [0, 1.0]])

Q = np.diag([2.0, 2.0, 5.0, 5.0])

def kalman_filter(pos_x_meas, pos_y_meas, speed_x_meas, speed_y_meas, x_est_prev, P_est_prev, dvx = 0, dvy = 0 ,obstructed = False, HT=None, HNT=None, RT=None, RNT=None):
    """
    Estimates the current state using input sensor data and the previous state
    
    param speed: measured speed (Thymio units)
    param ground_prev: previous value of measured ground sensor
    param ground: measured ground sensor
    param pos_last_trans: position of the last transition detected by the ground sensor
    param x_est_prev: previous state a posteriori estimation
    param P_est_prev: previous state a posteriori covariance
    
    return pos_last_trans: updated if a transition has been detected
    return x_est: new a posteriori state estimation
    return P_est: new a posteriori state covariance
    """
    
    ## Prediciton through the a priori estimate
    # estimated mean of the state
    U_in = [np.array([[dvx], [dvy]])]
    x_est_a_priori = A @ x_est_prev + B @ U_in
    # print("X_est_a_priori: {} ".format(x_est_a_priori))
    # print("old_X_est_prev: {} ".format(x_est_prev))
    
    # Estimated covariance of the state
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
    
    ## Update         
    # y, C, and R for a posteriori estimate, depending on transition

    if obstructed:
        R = np.diag([math.inf, math.inf, math.inf, math.inf])
    else:
        R = np.diag([2.0, 2.0, 5.0, 5.0])
    y = np.array([[pos_x_meas],[pos_y_meas],[speed_x_meas],[speed_y_meas]])
    H = np.diag([1,1,1,1])

    # innovation / measurement residual
    #i = y - np.dot(H, x_est_a_priori)
    i = y - H @ x_est_a_priori
    # print("Innovation: {} ".format(i))
    # measurement prediction covariance
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
    # print("S: {} ".format(S))
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))
    # print("Gain: {} ".format(K))
    
    # a posteriori estimate
    x_est = x_est_a_priori + K @ i
    # print("new_X_est_prev: {} ".format(x_est))
    P_est = P_est_a_priori - np.dot(K,np.dot(H, P_est_a_priori))
     
    return x_est, P_est, x_est_a_priori

cam = init_camera()
M, rect_width, rect_height, map, map_enlarged = init_map(cam)
thymio_found = False
thymio = MyThymio()
thymio.stop_thymio()
while not thymio_found:
    img, img_taken = take_picture(cam)
    print(img_taken)
    if img_taken:
        img_taken = False
        img_rect = get_rectified_img(img, M, rect_width, rect_height)
        thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
x_est = [np.array([[[thymio_pos[0]],[thymio_pos[1]], [0], [0]]])]
x_est_a_priori = [np.array([[[0],[0], [0], [0]]])]
print(x_est)
P_est = [1000 * np.ones(4)]
thymio.set_motor_speeds(100,100)
for k in range(50):
    #print(k)
    dvx = 0
    dvy = 0 
    img, img_taken = take_picture(cam)
    if img_taken:
        img_taken = False
        img_rect = get_rectified_img(img, M, rect_width, rect_height)
        thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        speed = (thymio.get_motor_left_speed()+thymio.get_motor_right_speed()) / 2 * 0.25
        if k == 25:
            dvx = -x_est_a_priori[-1][0][2][0]
            dvy = -x_est_a_priori[-1][0][3][0]
            thymio.stop_thymio()
            # print(x_est)
            print(dvx) 
            print(dvy)

        if k == 26:
            thymio.set_motor_speeds(100,100)
            dvx = 100 * math.cos(thymio.get_last_angle()) * 0.25
            dvy = - 100 * math.sin(thymio.get_last_angle()) * 0.25
            # print(x_est[-1][0][2][0])
            # print(x_est[-1][0][3][0])
            # print(thymio.get_last_angle())
            # print(dvx)
            # print(dvy)
        if k == 27:
            dvx = 0
            dvy = 0
        if thymio_found:
            thymio.set_last_angle(thymio_pos[2])
            thymio_found = False
            v_x = speed * math.cos(thymio.get_last_angle())
            v_y = - speed * math.sin(thymio.get_last_angle())
            # print(x_est[-1])
            new_x_est, new_P_est, new_x_est_a_priori = kalman_filter(thymio_pos[0], thymio_pos[1], v_x, v_y, x_est[-1], P_est[-1], dvx, dvy)
            x_est.append(new_x_est)
            P_est.append(new_P_est)
            x_est_a_priori.append(new_x_est_a_priori)
            time.sleep(0.1)
        else:
            thymio_found = False
            # print(x_est[-1])
            new_x_est, new_P_est, new_x_est_a_priori = kalman_filter(0, 0, 0, 0, x_est[-1], P_est[-1], dvx, dvy, obstructed = True)
            x_est.append(new_x_est)
            P_est.append(new_P_est)
            x_est_a_priori.append(new_x_est_a_priori)
            time.sleep(0.1)
        if k == 25:
            thymio.rotate_thymio(math.pi/3)


thymio.stop_thymio()
print("coucou")
print(x_est)
plt.figure()
plt.plot([x[0][0][0] for x in x_est], [y[0][1][0] for y in x_est], ".b")
plt.plot([x[0][0][0] for x in x_est_a_priori], [y[0][1][0] for y in x_est_a_priori], ".r")
plt.xlim([0, rect_width])
plt.ylim([0, rect_height])
plt.gca().invert_yaxis()
plt.show()




