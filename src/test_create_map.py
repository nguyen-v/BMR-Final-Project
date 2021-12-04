from camera import *
import cv2
from generate_aruco_markers import *

# https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

cam = init_camera()
aruco_dict = arucoDict = cv2.aruco.Dictionary_get(DEF_ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters_create()
while True:
    img, img_taken = take_picture(cam)
    if img_taken:
        # cv2.imshow('raw image', img)
        # cv2.waitKey(1)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
        # verify *at least* one ArUco marker was detected
        if len(corners) > 3:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                # show the output image
            cv2.imshow("Image", img)
            cv2.waitKey(2)
        else:
            print("aruco markers not found")
# NEEDS OPEN-CV-CONTRIB-PYTHON

