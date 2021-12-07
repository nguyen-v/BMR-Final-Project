import cv2
import time
from MyThymio import *
from camera import *
from create_map import *
from local_navigation import *
from locate_thymio_goal import *
import matplotlib.pyplot as plt
import math
from astar import *
from rdp import rdp

DELTA_T = 0.1
ANGLE_THRESHOLD = np.deg2rad(15)

# ========================================================================== #
#  Main function.                                                            # 
# ========================================================================== #

# def main():

#     cam = init_camera()
#     M, rect_width, rect_height, map, map_enlarged = init_map(cam)
#     obj_pos = [0, 0]
#     obj_found = False
#     while not obj_found:
#         img, img_taken = take_picture(cam)
#         if img_taken:
#             img_rect = get_rectified_img(img, M, rect_width, rect_height)
#             obj_pos, obj_found = locate_goal_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
#     thymio = MyThymio(verbose = True)
#     local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height)
            
# if __name__=="__main__":
#     main()

T_s = 0.1
A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0, 0, 0, 1.0]])
B = np.array([[T_s, 0], [0, T_s], [1.0, 0], [0, 1.0]])
Q = np.diag([5, 5, 10, 10])

def lin_refine_implicit(x, n):
    """
    Given a 2D ndarray (npt, m) of npt coordinates in m dimension, insert 2**(n-1) additional points on each trajectory segment
    Returns an (npt*2**(n-1), m) ndarray
    """
    if n > 1:
        m = 0.5*(x[:-1] + x[1:])
        if x.ndim == 2:
            msize = (x.shape[0] + m.shape[0], x.shape[1])
        else:
            raise NotImplementedError

        x_new = np.empty(msize, dtype=x.dtype)
        x_new[0::2] = x
        x_new[1::2] = m
        return lin_refine_implicit(x_new, n-1)
    elif n == 1:
        return x
    else:
        raise ValueError

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
    # print(y)
    H = np.diag([1,1,1,1])

    # innovation / measurement residual
    i = y - H @ x_est_a_priori

    # measurement prediction covariance
    S = H @ (P_est_a_priori @ H.T) + R
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = P_est_a_priori @ (H.T @ np.linalg.inv(S))


    # a posteriori estimate
    # print("a priori")
    # print(x_est_a_priori)
    # print("measured")
    # print(y)
    # print("K")
    # print(K)
    # print("Ki")
    # print(K @ i)
    x_est = x_est_a_priori + K @ i
    # print("a posteriori")
    # print(x_est)
    
    P_est = P_est_a_priori - K @ (H @ P_est_a_priori)
    # print("a post")
    # print(x_est[0:2])
    # print(y)
    # print("vx {}".format(x_est[2]))
    # print("vy {}".format(x_est[3]))
    return x_est, P_est, x_est_a_priori

def dist(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

# Connect to Thymio
thymio = MyThymio(verbose = True)
thymio.stop_thymio()

# Initialize camera
print("Initializing camera")
cam = init_camera()

# Initialize map
print("Initializing map")
M, rect_width, rect_height, map, map_enlarged = init_map(cam)

img, img_taken = take_picture(cam)

plt.figure()
plt.imshow(map, origin = 'lower', cmap = 'Greys', interpolation = 'nearest')
plt.title("Original Map")
plt.gca().invert_yaxis()
plt.show()

found_path = False
thymio_pos = []
obj_pos = []
path = []
while not found_path:
    # Find Thymio
    thymio_found = False
    while not thymio_found:
        img, img_taken = take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
    thymio.set_last_angle(thymio_pos[2])
    print("Thymio found.")

    # Find objective
    obj_found = False
    while not obj_found:
        img, img_taken = take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            obj_pos, obj_found = locate_goal_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))

    print("Objective found")
    # Compute global path to objective
    print("Computing global path")
    thymio_pos_grid = cartesian_to_grid(thymio_pos[0:2], (rect_width, rect_height), (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
    obj_pos_grid = cartesian_to_grid(obj_pos, (rect_width, rect_height), (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
    path, found_path = get_global_path(map_enlarged, thymio_pos_grid, obj_pos_grid)
    if not found_path:
        time.sleep(1)
        M, rect_width, rect_height, map, map_enlarged = init_map(cam)
print("Global path computed")

# Simplify path
path = rdp(path, epsilon=1)

# Add intermediate points to path 
path = lin_refine_implicit(path, n=2)


# Convert path to a list of (x, y) positions
path_temp = path + 1/2
path[:, 0] = path_temp[:, 1]*(rect_height/MAP_HEIGHT_CELL)
path[:, 1] = path_temp[:, 0]*(rect_width/MAP_WIDTH_CELL)


# Initialize a posteriori pose estimate
# The states are [x, y, vx, vy]
# x_est is a list of a posteriori estimates
x_est = [np.array([thymio_pos[0], thymio_pos[1], 0, 0])]

# Initialize a priori estimate
# x_est_a_priori is a list of a priori estimates
x_est_a_priori = x_est

last_node = x_est[-1][0:2]

# Initialize a posteriori covariance matrix of predicted state
# P_est is a list of covariance matrices
P_est = [1000 * np.eye(4)]                                                                  #

next_node_reached = True
go_to_next_node = False
# Initialize control inputs
dvx = 0
dvy = 0
while True:
    img_rect = np.zeros((rect_height, rect_width))
    img, img_taken = take_picture(cam)
    if img_taken:
        img_rect = get_rectified_img(img, M, rect_width, rect_height)

    # cv2.imshow('Rectified image', img_rect) 
    # cv2.waitKey(1)

    # Update measurements
    img, img_taken = take_picture(cam)
    obstructed = False
    if img_taken:
        img_rect = get_rectified_img(img, M, rect_width, rect_height)
        thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))

        speed = (thymio.get_motor_left_speed() + thymio.get_motor_right_speed())/2 * SPEED_COEFF
        x_meas = 0
        y_meas = 0
        vx_meas = 0
        vy_meas = 0
        # If Thymio found, update x, y, vx and vy measured
        if thymio_found:
            # thymio.set_last_angle(thymio_pos[2])
            # if (abs(thymio_pos[2] - thymio.get_last_angle()) < np.deg2rad(20)):
            thymio.set_last_angle(thymio_pos[2])

            # print(np.rad2deg(thymio_pos[2]))
            x_meas = thymio_pos[0]
            y_meas = thymio_pos[1]

            cv2.circle(img_rect, [int(x_meas), int(y_meas)] , 4, (0, 0, 255), -1)

            vx_meas = speed * math.cos(thymio.get_last_angle())
            vy_meas = -speed * math.sin(thymio.get_last_angle())
            print("meas")
            print([x_meas, y_meas, vx_meas, vy_meas])
        else:
            obstructed = True
    else:
        obstructed = True
    
    # Update a posteriori estimates
    new_x_est, new_P_est, new_x_est_a_priori = kalman_filter(x_meas, y_meas, vx_meas, vy_meas, x_est[-1], P_est[-1], dvx, dvy, obstructed = obstructed)
    x_est.append(new_x_est)
    P_est.append(new_P_est)
    # x_est_a_priori.append(new_x_est_a_priori)
    dvx = 0
    dvy = 0
    # Detect if next node reached or if Thymio has travelled the required distance
    # print("distance to next node {}".format(dist(path[0], x_est[-1][0:2])))
    # print("distance travelled {} out of {}".format(dist(last_node, x_est[-1][0:2]), dist(last_node, path[0])))
    # if ((dist(last_node, x_est[-1][0:2]) > dist(last_node, path[0])) and dist(path[0], x_est[-1][0:2]) < 50) or dist(path[0], x_est[-1][0:2]) < 10:
    if dist(path[0], x_est[-1][0:2]) < 10:
        next_node_reached = True
    # Compute the control inputs to go to next node
    if go_to_next_node:
        dvx = BASE_SPEED*math.cos(thymio.get_last_angle()) * SPEED_COEFF - x_est_a_priori[-1][2]
        dvy = -BASE_SPEED*math.sin(thymio.get_last_angle()) * SPEED_COEFF -x_est_a_priori[-1][3]
        thymio.set_motor_speeds(BASE_SPEED, BASE_SPEED)
        go_to_next_node = False

#     # Rotate the Thymio towards next node if current node reached
    if next_node_reached:
        last_node = path[0]
        path = np.delete(path, 0, 0)
        if len(path) == 0:
            thymio.stop_thymio()
            break # we have reached the last node
        next_node = path[0]
        thymio_angle = thymio.get_last_angle()
        img, img_taken = take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
            if thymio_found:
                thymio_angle = thymio_pos[2]
        th_obj_angle = math.atan2(-(next_node[1] - x_est[-1][1]), next_node[0] - x_est[-1][0])
        print("angle between next node and thymio {}".format(th_obj_angle))
        da = thymio_angle - th_obj_angle
        print("thymio angle \t {}".format(np.rad2deg(thymio.get_last_angle())))
        print("da \t {}".format(da))
        if abs(da) > ANGLE_THRESHOLD:
            print("angle too big")
            if da > 0:
                if da < math.pi:
                    da = -da
                if da > math.pi:
                    da = 2*math.pi - da
            elif da < 0:
                if da > -math.pi:
                    da = -da
                if da < -math.pi:
                    da = -2*math.pi - da

            # Compute the control inputs that to stop the motors (dvx = -vx_a_priori, dvy = -vy_a_priori)
            # dvx = -x_est_a_priori[-1][2]
            # dvy = -x_est_a_priori[-1][3]
            thymio.stop_thymio()
            thymio.rotate_thymio(-da)
        go_to_next_node = True
        next_node_reached = False
    
    print("x_est end")
    print(x_est[-1])
    cv2.arrowedLine(img_rect, (int(x_est[-1][0]), int(x_est[-1][1])), (int(x_est[-1][0] + math.cos(thymio.get_last_angle())*50), int(x_est[-1][1] - math.sin(thymio.get_last_angle())*50)),
                                (128, 0, 255), 3, tipLength = 0.3)
    cv2.polylines(img_rect, np.int32([path]), False, (255, 0, 255), 3)
    cv2.circle(img_rect, [int(x_est[-1][0]),int(x_est[-1][1])] , 8, (0, 255, 255), -1)
    cv2.imshow('Rectified image', img_rect) 
    cv2.waitKey(1)

    # Regulate along the path to reach next node
    img, img_taken = take_picture(cam)
    if img_taken:
        img_rect = get_rectified_img(img, M, rect_width, rect_height)
        thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
        if thymio_found:
            a_th_obj = angle_two_points(x_est[-1][0], x_est[-1][1], path[0][0], path[0][1])
            err_a = thymio.get_last_angle() - a_th_obj
            print("thymio angle \t {}".format(x_est[-1][2]))
            print("angle th ob \t {}".format(a_th_obj))
            print("error \t {}".format(err_a))
            if (err_a < -math.pi):
                err_a = err_a + 2*math.pi
            if (err_a > math.pi):
                err_a = err_a - 2*math.pi
            thymio.set_motor_left_speed(int(BASE_SPEED + 4*err_a/math.pi*BASE_SPEED))
            thymio.set_motor_right_speed(int(BASE_SPEED - 4*err_a/math.pi*BASE_SPEED))
    time.sleep(T_s)                                                                                                                             #

# print("Plotting")
# plt.figure()
# plt.plot([x[0] for x in x_est], [y[1] for y in x_est], ".b")
# plt.plot([x[0] for x in x_est_a_priori], [y[1] for y in x_est_a_priori], ".r")
# plt.xlim([0, rect_width])
# plt.ylim([0, rect_height])
# plt.gca().invert_yaxis()
# plt.show()



