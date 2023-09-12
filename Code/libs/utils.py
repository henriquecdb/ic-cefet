import time
import numpy as np
from math import *


def approx_Equal(x, y, tolerance=0.001):
    return abs(x-y) <= 0.5 * tolerance * (x + y)


def collinear(x1, y1, x2, y2, x3, y3):
    a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
    print(a)
    if (a <= 0.5 and a >= 0):
        return True
    elif (a >= -0.5 and a <= 0):
        return True
    return False


def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi


def xdot(th, r, L):
    return np.array([r*cos(th)/2, r*cos(th)/2], [r*sin(th)/2, r*sin(th)/2], [r/L, -r/L])


def distanceBetweenPoints(qRobot, qGoal):
    return np.sqrt((qGoal[0] - qRobot[0]) ** 2 + (qGoal[1] - qRobot[1]) ** 2)
