## 
# @file main.py
#
# @brief Definition of main file for global and local navigation of a Thymio.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import time
import math

## Custom modules
from MyThymio import *
from camera import *
from create_map import *
from local_navigation import *
from locate_thymio_goal import *
from global_path_planning import *
from kalman_filter import *

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Angle threshold before the thymio tries to rectifiy its rotation.
ANGLE_THRESHOLD = np.deg2rad(15)

## Proximity sensors threshold to trigger local navigation.
LOCAL_NAV_PROX_THR = 3700

## Minimum distance to move objective before objective "kidnapping" is detected.
OBJ_KIDNAPPING_THR = 100

## Minimum distance to move thymio before robot "kidnapping" is detected.
THYMIO_KIDNAPPING_THR = 200

## Obstacle size in pixels. Larger values will make the local avoidance aim
#  for a node that is further away.
OBST_SIZE = 150

## Time before path recalculation after detection of objective moved.
OBJ_MOVED_DELTA_T = 1

## Initial estimated covariance
INIT_COV = 1000

# ========================================================================== #
#  Main function.                                                            # 
# ========================================================================== #

def main():
    # Connect to Thymio
    thymio = MyThymio(verbose = True)
    thymio.stop_thymio()

    # Initialize camera
    print("Initializing camera")
    cam = init_camera()

    # Initialize map
    print("Initializing map")
    M, rect_width, rect_height, map, map_enlarged = init_map(cam)

    # Initialize path
    path, thymio_pos, obj_pos = init_path(cam, thymio)

    # Initialize a posteriori pose estimate
    # The states are [x, y, vx, vy]
    # x_est is a list of a posteriori estimates
    x_est = [np.array([thymio_pos[0], thymio_pos[1], 0, 0])]

    # Initialize a posteriori covariance matrix of predicted state
    # P_est is a list of covariance matrices
    P_est = [INIT_COV * np.eye(4)]

    # Initialization of some variables
    next_node_reached = True
    has_rotated = False
    go_to_next_node = False
    obj_moved = False
    thymio_moved = False
    kidnapping_timestamp = 0

    # Initialize control inputs
    dvx = 0
    dvy = 0

    # ---------------------------------------------------------------------- #     
    #  Main loop.                                                            # 
    # ---------------------------------------------------------------------- #

    while True:
        start_time = time.time()
        img_rect = np.zeros((rect_height, rect_width))

        # Update measurements if possible
        img, img_taken = take_picture(cam)
        obstructed = False
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            x_meas, y_meas, vx_meas, vy_meas, obstructed = thymio.get_measurements(img_rect)
        else:
            obstructed = True
        
        # Update a posteriori estimates (Kalman filter)
        new_x_est, new_P_est = kalman_filter(x_meas, y_meas, vx_meas, vy_meas, x_est[-1], 
                                             P_est[-1], dvx, dvy, obstructed = obstructed)
        thymio.set_last_position(new_x_est[0:2])
        x_est.append(new_x_est)
        P_est.append(new_P_est)

        # Set default control inputs to 0
        dvx = 0
        dvy = 0

        # Detect if next node reached
        if dist(path[0], x_est[-1][0:2]) < NODE_DIST_THR:
            next_node_reached = True

        # Compute the control inputs to go to next node
        if go_to_next_node:
            # we have to send control inputs that will make it go to the desired vx, vy
            # The control inputs are divided into 2 terms: one to stop the motors in the model
            # and one to have the correct vx, vy once the Thymio has rotated.
            dvx = BASE_SPEED*math.cos(thymio.get_last_angle()) * SPEED_COEFF - x_est[-1][2]
            dvy = -BASE_SPEED*math.sin(thymio.get_last_angle()) * SPEED_COEFF -x_est[-1][3]
            thymio.set_motor_speeds(BASE_SPEED, BASE_SPEED)
            go_to_next_node = False

        # Rotate the Thymio towards next node if current node reached
        if next_node_reached:
            path = np.delete(path, 0, 0)

            if len(path) == 0:
                thymio.stop_thymio()
                cv2.destroyWindow('Rectified image')
                print("Global objective reached")
                break

            next_node = path[0]
            thymio_angle = thymio.get_last_angle()
            th_obj_angle = angle_two_points(x_est[-1][0], x_est[-1][1], next_node[0], next_node[1])
            da = thymio_angle - th_obj_angle

            if abs(da) > ANGLE_THRESHOLD and dist(path[0], x_est[-1][0:2]) > NODE_DIST_THR/2:
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
                thymio.stop_thymio()
                has_rotated = True
                thymio.rotate_thymio(-da)
            else:
                has_rotated = False
            go_to_next_node = True
            next_node_reached = False

        # Regulate along the path to reach next node
        img, img_taken = take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            thymio_pos, thymio_found = locate_thymio_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
            if thymio_found:
                a_th_obj = angle_two_points(x_est[-1][0], x_est[-1][1], path[0][0], path[0][1])
                err_a = thymio.get_last_angle() - a_th_obj
                if (err_a < -math.pi):
                    err_a = err_a + 2*math.pi
                if (err_a > math.pi):
                    err_a = err_a - 2*math.pi
                thymio.set_motor_left_speed(int(BASE_SPEED + 4*err_a/math.pi*BASE_SPEED))
                thymio.set_motor_right_speed(int(BASE_SPEED - 4*err_a/math.pi*BASE_SPEED))
        else:
            thymio.set_motor_speeds(BASE_SPEED, BASE_SPEED)

        # Check if global objective position has moved. Recalculate path if so.
        if img_taken:
            new_obj_pos, obj_found = locate_goal_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
            if obj_found and len(obj_pos) != 0:
                if dist(obj_pos, new_obj_pos) >  OBJ_KIDNAPPING_THR:
                    obj_moved = True
                    obj_pos = new_obj_pos
                    kidnapping_timestamp = time.time()
                    print("Global objective has moved.")
            if len(x_est) > 1:
                if dist(x_est[-1][0:2], x_est[-2][0:2]) > THYMIO_KIDNAPPING_THR:
                    thymio_moved = True
                    thymio_pos = new_obj_pos
                    kidnapping_timestamp = time.time()
                    print("Thymio has been kidnapped.")
                
        # Wait a bit before recalculating path. We want to leave some time for the user to move
        if (obj_moved or thymio_moved) and time.time() - kidnapping_timestamp > OBJ_MOVED_DELTA_T:
                obj_moved = False
                thymio_moved = False
                thymio.stop_thymio()
                print("Recalculating path")
                obj_pos = new_obj_pos
                path = init_path(cam, thymio, clear_start_node = True, use_last_pos = True)[0]
                next_node_reached = True
        
        # Display Thymio on camera feed
        cv2.arrowedLine(img_rect, (int(x_est[-1][0]), int(x_est[-1][1])), (int(x_est[-1][0] + 
                        math.cos(thymio.get_last_angle())*50), int(x_est[-1][1] - math.sin(thymio.get_last_angle())*50)),
                        (128, 0, 255), 3, tipLength = 0.3)
        cv2.polylines(img_rect, np.int32([path]), False, (255, 0, 255), 3)
        cv2.circle(img_rect, [int(x_est[-1][0]),int(x_est[-1][1])] , 8, (0, 255, 255), -1)

        # Local avoidance
        prox_values = thymio.get_prox_horizontal()
        if any(prox > LOCAL_NAV_PROX_THR for prox in prox_values):
            local_objective = []
            if len(path) != 0:
                for idx, node in enumerate(path):
                    if dist(x_est[-1][0:2], node) > OBST_SIZE:
                        local_objective = node
                        path = np.delete(path, np.arange(idx), 0)
                        break
                if len(local_objective) == 0:
                    local_objective = path[-1]
                    path = [path[-1]]
                try:
                    cv2.destroyWindow('Rectified image')
                except: # might raise error if this destroywindow is called before a window has been created
                    pass
                local_avoidance(thymio, local_objective, cam, M, rect_width, rect_height, map_enlarged)

        cv2.imshow('Rectified image', img_rect) 
        cv2.waitKey(1)
        
        # We want to take into account execution time between each Kalman update call
        end_time = time.time()
        delta_time = end_time - start_time
        if delta_time < T_s:
            update_sampling_time(T_s)
            time.sleep(T_s-delta_time)     
        elif not has_rotated: # rotate function has a sleep inside, which would interfere with time measures
            #update sampling time if execution time too slow (happens if we are recording at the same time)
            update_sampling_time(delta_time)     

if __name__=="__main__":
    main()