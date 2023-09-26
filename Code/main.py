#!/usr/bin/env python
from src.Follower import *
from coppeliasim_zmqremoteapi_client import *


def main():
    robot_2 = RobotMQ(robotname="Pioneer_p3dx_2", maxv=0.3)
    robot_2.qgoal = [2, 3, 0]

    robot_1 = Follower(robotname="Pioneer_p3dx_1", maxv=0.4)
    # robot_1.qgoal = [2, -3, 0]

    robot_3 = Follower(robotname="Pioneer_p3dx_3", maxv=0.4)
    # robot_1.qgoal = [2, -3, 0]

    while (True):
        r2 = robot_2._run_step()
        r1 = robot_1._follow_step(robot_2)
        r3 = robot_3._follow_step(robot_1)
        if (r1 and r2 and r3):
            break

    robot_2._stop()
    robot_1._stop()
    robot_3._stop()


if __name__ == '__main__':
    main()
