import numpy as np
from rdp import rdp
import matplotlib.pyplot as plt

def lin_refine_implicit(x, n):
    """
    Given a 2D ndarray (npt, m) of npt coordinates in m dimension, insert 2**(n-1) additional points on each trajectory segment
    Returns an (npt*2**(n-1), m) ndarray
    """
    if n > 1:
        m = 0.5*(x[:-1] + x[1:])
        if x.ndim == 2:
            msize = (x.shape[0] + m.shape[0], x.shape[1])
        else:
            raise NotImplementedError

        x_new = np.empty(msize, dtype=x.dtype)
        x_new[0::2] = x
        x_new[1::2] = m
        return lin_refine_implicit(x_new, n-1)
    elif n == 1:
        return x
    else:
        raise ValueError

path = np.array([[0, 0], [0, 50], [1, 100], [0, 150], [0, 200], [50, 200], [100, 250], [150, 300], [200, 250], [250, 200], [300, 150]])
# print(path)
# # x, y = path.T
# plt.figure()
# # plt.scatter(x,y)
# plt.plot(path[:,0], path[:,1], 'ob', ms=15.0, label='original data')
# plt.show()
# plt.show()
# path_simplified = rdp(path, epsilon=10)
# plt.figure()
# plt.plot(path_simplified[:,0], path_simplified[:,1], 'ob', ms=15.0, label='after simplification data')
# plt.show()

# path_interp = lin_refine_implicit(path_simplified, n=3)
# plt.figure()
# plt.plot(path_interp[:,0], path_interp[:,1], 'ob', ms=15.0, label='after adding points data')
# plt.show()

# path = np.array([np.array([[0], [0]]), np.array([[100], [0]]), np.array([[200], [200]]), np.array([[200], [100]])])
# print(path)
# print("a")


# path = [np.array([[[1],[2],[3]]]), np.array([[[4],[5],[6]]])]
# # print([x[0][0] for x in path])
# # print([x[1][0] for x in path])
# print([x[0][0][0] for x in path])
# print([x[0][1][0] for x in path])
# print(path)
# plt.plot([x[0][0][0] for x in path], [y[0][1][0] for y in path])
# plt.show()

# test = np.array([0, 1, 2])
# test2 = [0, 1, 2]
# print(test[1:2])
# print(test2[0:2])
# T_s = 0.1
# A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0,0,0,1.0]])
# x = np.array([1, 2, 3, 4])
# print(1000*np.eye(4))

# path = np.array([[1, 2], [3,4], [5, 6]])
# print(path[0])
# path = np.delete(path, 0, 0)
# print(path)
# print(len(path))
# x = [np.array([1, 2, 3, 4])]
# y = np.array([5, 6, 7, 8])
# x.append(y)
# print(x[-1][2])
# path = path + 1/2
# print(path)
# path[:,0] = 5*path[:, 0]
# print(path)
# x_est = [np.array([1, 2, 3, 4]), np.array([2, 4, 3, 4]), np.array([0, 1, 3, 4])]
# print(x_est[-1][0:2])
# plt.figure()
# plt.plot([x[0] for x in x_est], [y[1] for y in x_est], ".b")
# plt.xlim([0, rect_width])
# plt.ylim([0, rect_height])
# plt.gca().invert_yaxis()
# plt.show()
# for x in x_est:
#     print(x[0])
#     print(x[1])

x_est = [np.array([1, 2]), np.array([3, 4])]
new_x_est = np.array([5, 7])
print(new_x_est)
x_est.append(new_x_est)
print(x_est[-1])