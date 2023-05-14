#!/usr/bin/env python
from bugs.PotentialFields import *


def main(x, y, th):
    new_potential = PotentialFields(x, y, th)
    new_potential._run()


if __name__ == '__main__':
    main(3, 0, 0)
