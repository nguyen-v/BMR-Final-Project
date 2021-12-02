# from thymio_connection import connect_to_thymio
# from MyThymio import *
# import time

# thymio = MyThymio(verbose = True)

# thymio.set_motor_left_speed(50)
# time.sleep(1)
# thymio.stop_thymio()

import cv2
import time
from MyThymio import *
from camera import *
from create_map import *
from local_navigation import *
from locate_thymio_goal import *
import matplotlib.pyplot as plt

DELTA_T = 0.1

# ========================================================================== #
#  Main function.                                                            # 
# ========================================================================== #


def main():

    # Initialisations
    cam = init_camera()
    # thymio = MyThymio()
    success = False
    while (success == False):
        img, img_taken = take_picture(cam)
        if img_taken:
            M, rect_width, rect_height, map, map_enlarged, success = create_map(img, 11, 7)
            # cv2.imshow('Window', img)
            # cv2.waitKey(1)
        else:
            print("img not taken")
        # print(img.shape)
        # M, rect_width, rect_height, map, map_enlarged, success = create_map(img, 7, 11)
        time.sleep(DELTA_T)
    plt.figure()
    plt.imshow(map, origin = 'lower')
    plt.title("Original Map")
    plt.gca().invert_yaxis()
    plt.show()
    # # local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height)
    while True:
        img, img_taken= take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width,  rect_height)
            thymio_pose, thymio_found = locate_thymio_camera(img_rect, "cartesian", (11, 7))
            # print(thymio_found)
            if thymio_found:
                print("Thymio found: x: {} \t y: {} \t theta: {}".format(thymio_pose[0], thymio_pose[1], thymio_pose[2]))
            cv2.imshow('Window', img_rect)
            cv2.waitKey(1)

if __name__=="__main__":
    main()