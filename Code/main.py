#!/usr/bin/env python
from src.PotentialFields import *


def main(x, y, th):
    new_potential = PotentialFields(x, y, th)
    new_potential._run()


if __name__ == '__main__':
    # x, y, th
    main(0, 4, 0)
