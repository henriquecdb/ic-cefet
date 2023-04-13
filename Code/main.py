#!/usr/bin/env python

from Robot import *
import numpy as np

if __name__ == '__main__':

    bug = Robot('Pioneer_p3dx', 0.331, 0.09751, 1.0, np.deg2rad(45), False)
    bug.run()