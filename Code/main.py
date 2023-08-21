#!/usr/bin/env python
from src.Follower import *


def main():
    robot_leader = Robot(robotname="Pioneer_p3dx_2")
    robot_follower = Follower()
    robot_follower._follow(robot_leader)


if __name__ == '__main__':
    main()
