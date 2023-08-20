#!/usr/bin/env python
from src.Leader import *


def main(x, y, th):
    new_robot = Leader(x, y, th)
    new_robot.run()


if __name__ == '__main__':
    main(2, 4, 0)
