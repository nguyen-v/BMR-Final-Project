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
def local_avoidance2(obj_pos, cam, M, rect_width, rect_height):
    while True:
        img, img_taken= take_picture(cam)
        print(img_taken)
        if img_taken:
            img_rect = get_rectified_img(img, M, rect_width,  rect_height)
            cv2.imshow('Window', img_rect)
            cv2.waitKey(1)
            thymio_pose, found_thymio = locate_thymio_camera(img_rect, "cartesian", (11, 7))
            print("found thymio: {}".format(found_thymio))
            print(thymio_pose)
            if found_thymio:
                y = np.zeros(2)
                
                # Local objective contribution (attractive)
                a_th_obj = angle_two_points(thymio_pose[0], thymio_pose[1], obj_pos[0], obj_pos[1])
                da = thymio_pose[2] - a_th_obj
                if (da < -math.pi):
                    da = da + 2*math.pi
                print("delta angle {}".format(da))
                y[0] = y[0] + OBJ_ATT_BASE + da/math.pi*OBJ_ATT_COEFF
                y[1] = y[1] + OBJ_ATT_BASE - da/math.pi*OBJ_ATT_COEFF

                # # Map obstacles contribution (repulsive)
                # gnd_sens = np.divide(thymio.get_gnd_sensors(), GND_SCALE)
                # we have to put a threshold on ground sensors if global obstacles have no gradient
                # for i in range(NUM_GND_SENS):
                #     y[0] = y[0] + gnd_sens[i] * WEIGHTS_LEFT_GND[i]
                #     y[1] = y[1] + gnd_sens[i] * WEIGHTS_RIGHT_GND[i]
                if math.dist([thymio_pose[0], thymio_pose[1]], [obj_pos[0], obj_pos[1]]) < LOC_DIST_THR:
                    break
            time.sleep(0.1)
        time.sleep(0.1)

def main():

    # Initialisations
    cam = init_camera()
    success = False
    M = []
    rect_height = 0
    rect_width = 0
    while (success == False):
        img, img_taken = take_picture(cam)
        
        if img_taken:
            M, rect_width, rect_height, map, map_enlarged, success = create_map(img, 11, 7)
        else:
            print("img not taken")
        time.sleep(DELTA_T)
        print("map created")
    obj_pos = [100, 100]
    thymio = MyThymio(verbose = True)
    local_avoidance(thymio, obj_pos, cam, M, rect_width, rect_height)

if __name__=="__main__":
    main()
