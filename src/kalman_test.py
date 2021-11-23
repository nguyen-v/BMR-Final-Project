import matplotlib.pyplot as plt
import numpy as np

from Kalman import kalman_filter as KF

DT = 0.1
main_filter = KF()
SIM_TIME = 500
show_animation = 1

def main():
    time = 0.0

    # State Vector [x y yaw v]'
    xPost = np.zeros((7, 1))
    xPred = np.zeros((7, 1))
    xTrue = np.zeros((2, 1))

    # history
    hxPost = xPost
    hxPred = xPred
    hxTrue = xTrue

    while SIM_TIME >= time:

        xPost,P_post,S_post = main_filter.filter()
        
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
