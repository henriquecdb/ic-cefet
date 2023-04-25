#!/usr/bin/env python

from bugs.Bug import *
from bugs.Robot import *
import numpy as np
import sys


def main(x, y, th):
    # bug = Robot('Pioneer_p3dx', 0.331, 0.09751, 1., np.deg2rad(45), False, 3, 3)
    # bug.run()

    newBug = Bug(x, y, th)
    newBug._bug1()
    # newBug._bug2()

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
