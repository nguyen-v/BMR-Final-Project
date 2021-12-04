import cv2
import time
from MyThymio import *
from camera import *
from create_map import *
from local_navigation import *
from locate_thymio_goal import *
import matplotlib.pyplot as plt
import math

DELTA_T = 0.1

# ========================================================================== #
#  Main function.                                                            # 
# ========================================================================== #

def main():

    cam = init_camera()
    M, rect_width, rect_height, map, map_enlarged = init_map(cam)
    obj_pos = [0, 0]
    obj_found = False
    while not obj_found:
        img, img_taken = take_picture(cam)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width, rect_height)
            obj_pos, obj_found = locate_goal_camera(img_rect, "cartesian", (MAP_WIDTH_CELL, MAP_HEIGHT_CELL))
    thymio = MyThymio(verbose = True)
    local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height)
            
if __name__=="__main__":
    main()
