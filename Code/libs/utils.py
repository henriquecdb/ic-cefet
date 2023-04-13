import time
import numpy as np

def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi

def collinear(x1, y1, x2, y2, x3, y3):
    a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
    print(a)
    if (a <= 0.5 and a >= 0):
        return True
    elif (a >= -0.5 and a <= 0):
        return True
    return False
