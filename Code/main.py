#!/usr/bin/env python
from src.Follower import *


def main():
    robot_leader = Robot(robotname="Pioneer_p3dx_2", leader=True)
    robot_leader.qgoal = [2, 4, 0]

    robot_follower = Follower(robot_leader)
    # robot_leader.run()


if __name__ == '__main__':
    main()
