## 
# @file kalman_test.py
#
# @brief Tests the behaviour of the kalman_filter

# ========================================================================== #
#  Imports.                                                                  # 
# ========================================================================== #

import matplotlib.pyplot as plt
import math
import numpy as np

from kalman import kalman_filter as KF
# from Kalman import extended_kalman as EKF

# ========================================================================== #
#  Global constants.                                                         # 
# ========================================================================== #

# Sampling time in seconds
DT = 0.1

main_filter = KF()
#yextended_filter = EKF()

# Simulation time in seconds
SIM_TIME = 999
show_animation = 1

# ========================================================================== #
#  Exported functions.                                                       # 
# ========================================================================== #

def main():
    time = 0.0
    # State Vectors
    xPost = np.zeros((4, 1))
    xPred = np.zeros((4, 1))
    xTrue = np.zeros((2, 1))  

    # History
    hxPost = xPost
    hxPred = xPred
    hxTrue = xTrue
    while SIM_TIME >= time:

        time = time + DT
        if time < 2: 
            main_filter.measurements_test(0.5,0.5)
        elif time == 2:
            main_filter.save_input_control(np.array([[-xPost[2,0]],[-xPost[3,0]]]))
            main_filter.measurements_test(0,0)
        elif time >= 2.1 and time <= 4:
            main_filter.measurements_test(-0.1,0.5)
            main_filter.save_input_control(np.array([[(-0.3+0.5)/2*math.cos(main_filter.angle_test)],[(-0.3+0.5)/2*math.sin(main_filter.angle_test)]]))
        elif time > 4:
            main_filter.measurements_test(0.5,0.5)
            main_filter.save_input_control(np.zeros((2,1)))

    #        xPost,P_post,S_post = main_filter.filter()
        
    #        xPred = main_filter.kalman_get_prediction()

    #        xTrue = main_filter.return_meas_test()

        xPost = main_filter.filter()
        
        xPred = main_filter.kalman_get_prediction()

        xTrue = main_filter.return_meas_test()
        
        # store data history
        hxPost = np.hstack((hxPost, xPost))
        hxPred = np.hstack((hxPred, xPred))
        hxTrue = np.hstack((hxTrue, xTrue))


        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hxPred[0, :].flatten(),
                        hxPred[1, :].flatten(), ".b")
            plt.plot(hxPost[0, :].flatten(),
                        hxPost[1, :].flatten(), "-r")
            plt.plot(hxTrue[0, :].flatten(),
                        hxTrue[1, :].flatten(), "-g")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

if __name__ == '__main__':
    main()
