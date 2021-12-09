## 
# @file camera.py
#
# @brief Definition of functions for camera control

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import cv2
import os

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## Path to file where camera images can be stored.
dirname = os.path.dirname(__file__)
dirname = os.path.join(dirname, '../img/cam/')

## Camera feed width in pixels.
IMAGE_WIDTH = 800

## Camera feed height in pixels.
IMAGE_HEIGHT = 600

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Initializes the camera.
#  @param   IMAGE_WIDTH     Desired camera feed image width.
#  @param   IMAGE_HEIGHT    Desired camera feed image height.
#  @return  cam             Camera instance.
def init_camera(width = IMAGE_WIDTH, height = IMAGE_HEIGHT):
    cam=cv2.VideoCapture(0 + cv2.CAP_DSHOW)
    cam.set(3,width)
    cam.set(4,height)
    return cam

## Returns a picture from camera.
#  @param   cam         Camera instance.
#  @return  img         Image from camera.
#  @return  img_taken   True if image was successfully taken.
def take_picture(cam):
        img_taken ,img = cam.read()
        if img_taken: 
            return img, img_taken
        else:
            return [], img_taken

## Saves a picture.
#  @param   img     Image to save.
def save_camera_img(img, filename):
    cv2.imwrite(dirname + filename + ".png", img)

cam = init_camera()

img_taken = False
while not img_taken:
    img_normal, img_taken = take_picture(cam)
save_camera_img(img_normal, "img_normal")