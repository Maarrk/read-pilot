import numpy as np
from math import atan
import math

def quad_center(q):
    """
    Finds diagonals intersection using determinants
    @param quad: 4 1 2 numpy array with quad vertices in order
    @return: tuple with point coordinates
    """
    q = np.float32(q)

    # x0y2-y0x2
    A = q[0][0][0] * q[0][2][1] - q[0][0][1] * q[0][2][0]
    # x3y1 - y3x1
    B = q[0][3][0] * q[0][1][1] - q[0][3][1] * q[0][1][0]
    # (x0-x2)(y3-y1)-(y0-y2)(x3-x1)
    C = (q[0][0][0] - q[0][2][0]) * (q[0][3][1] - q[0][1][1]) - (q[0][0][1] - q[0][2][1]) * (q[0][3][0] - q[0][1][0])

    # A(x3-x1)-B(x0-x2)
    px = A * (q[0][3][0] - q[0][1][0]) - B * (q[0][0][0] - q[0][2][0])
    # A(y3-y)-B(y0-y2)
    py = A * (q[0][3][1] - q[0][1][1]) - B * (q[0][0][1] - q[0][2][1])

    return px / C, py / C


def get_angles(pixel, mtx_inv):
    """
    Angles from main optical axis
    @param pixel: pixel coordinates of point
    @return: alpha to right, beta to bottom angles in radians
    """
    pixel = np.array([[pixel[0]], [pixel[1]], [1]])
    real = np.matmul(mtx_inv, pixel)
    # z is always 1 without scalar factor
    real_rad = atan(real[0][0]), atan(real[1][0])
    # print 'alpha: {:06.3}deg, beta: {:06.3}deg'.format(real_rad[0]*180/pi, real_rad[1]*180/pi)
    return real_rad


def get_direction(pixel, mtx_inv):
    pixel = np.array([[pixel[0]], [pixel[1]], [1]])
    real = np.reshape(np.matmul(mtx_inv, pixel),(1,3))
    front = np.matmul(real, np.array([[0,1,0],[0,0,1],[1,0,0]]))
    return front / np.sum(front*front)


def euler2rotmat(theta):
    """
    Calculate rotation matrix
    :param theta: roll, pitch, yaw
    :return: 3x3 rotation matrix
    """
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_x, np.dot(R_y, R_z))

    return R


if __name__ == "__main__":
    front = np.array([1,0,0])
    att = np.radians(np.array([0,90,0]))

    print np.matmul(front, euler2rotmat(att))