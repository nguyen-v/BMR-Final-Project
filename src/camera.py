## 
# @file camera.py
#
# @brief Definition of functions for camera control

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import time
import os

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../img/camera_shot.jpg')

img_ready = False

MAP_WIDTH = 640

MAP_LENGTH = 480

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

def init_camera(width = MAP_WIDTH, height = MAP_LENGTH):
    cam=cv2.VideoCapture(0 + cv2.CAP_DSHOW)
    cam.set(3,width)
    cam.set(4,height)
    return cam

def take_picture(cam, video = 0):
        ret_val,img = cam.read()
        # print("ret_val : {}, is_opened : {}.".format(ret_val,cam.isOpened()))
        if ret_val:
            cv2.imshow('Window', img)
            cv2.waitKey(1)
            save_camera_img(img)   
            return img 
                
def img_ready():
    global img_ready 
    img_ready = True

def save_camera_img(img):
    cv2.imwrite(filename, img)


# Easy implementation of camera
#cam = init_camera()
#while True:
    #img = take_picture(cam)
