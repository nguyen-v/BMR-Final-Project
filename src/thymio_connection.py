## 
# @file thymio_connection.py
#
# @brief Definition of functions to connect to Thymio.

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

from cv2 import TermCriteria_COUNT
from Thymio import Thymio
import serial                           
import serial.tools.list_ports # detection of serial ports
import time

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

## List of possible Thymio VIDs. This can be checked using check_vid().
THYMIO_VID = [1559]

## Number of tries to connect to Thymio.
MAX_NUM_ERR = float('inf')
# MAX_NUM_ERR = 10

## Serial connection with thymio refresh rate.
DEF_REFR_RATE = 0.1

## Time between each connection try.
DELTA_T_ERR = 1

## Cooldown wait time after connection.
DETTA_T_COOLDOWN = 3

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

## Displays all current port names and corresponding VID.
def check_vid():
    for p in serial.tools.list_ports.comports():
        print("Port: {}, VID: {}".format(p.name, p.vid))

## Returns a list of possible thymio ports.
#  @return      thymio_ports    List of possible Thymio ports, e.g. ['COM3'].
def get_thymio_port():
    thymio_ports = [
        p.name
        for p in serial.tools.list_ports.comports()
        if p.vid in THYMIO_VID
    ]
    return thymio_ports

## Initiates serial communication with the Thymio.
#  @param       num_err         Maximum number of errors before returning None.
#  @param       verbose         Set to True for debugging messsages.
#  @return      th              Object for serial communication.
def connect_to_thymio(num_err = MAX_NUM_ERR, verbose = False):
    error_cnt = 1
    while error_cnt <= num_err:
        if verbose:
            print("Connecting to Thymio. Try {}/{}...".format(error_cnt, num_err))
        thymio_port = get_thymio_port()
        if len(thymio_port) != 0:
            # If multiple Thymios are connected, just connect to the first one on the list.
            connected_to_thymio = False
            while not connected_to_thymio:
                th = Thymio.serial(port = thymio_port[0], refreshing_rate = DEF_REFR_RATE) 
                time.sleep(DETTA_T_COOLDOWN)
                connected_to_thymio = True
                if verbose:
                    print("Successfully connected to thymio on {}.".format(thymio_port[0]))
                return th

        else:
            error_cnt = error_cnt + 1
            time.sleep(DELTA_T_ERR)
    if verbose:
        print("Connection to Thymio unsucessful.")
    return None
