# from thymio_connection import connect_to_thymio
from MyThymio import *
import time

thymio = MyThymio(verbose = True)

thymio.set_motor_left_speed(50)
time.sleep(1)
thymio.stop_thymio()