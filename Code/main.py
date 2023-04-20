#!/usr/bin/env python

from bugs.Bug import *
from bugs.Robot import *
import numpy as np

if __name__ == '__main__':

    #bug = Robot('Pioneer_p3dx', 0.331, 0.09751, 1., np.deg2rad(45), False, 3, 3)
    #bug.run()

    newBug = Bug(4, 4, 90)
    newBug._bug1()
    #newBug._bug2()
