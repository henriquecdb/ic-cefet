#!/usr/bin/env python
from src.Follower import *
from coppeliasim_zmqremoteapi_client import *


def main():
    # robot_leader = RobotMQ(robotname="Pioneer_p3dx_2", leader=True, maxv=0.3, go=True, x=3,y=4,th=0)
    # robot_follower = RobotMQ(robotname="Pioneer_p3dx[0]", leader=True, maxv=0.3, go=True, x=-2,y=2,th=0)
    robot_1 = RobotMQ(robotname="Pioneer_p3dx", maxv=0.3)
    robot_1.qgoal = [2, -3, 0]

    robot_2 = RobotMQ(robotname="Pioneer_p3dx_2", maxv=0.3)
    robot_2.qgoal = [2, 3, 0]

    while (True):
        # r1 = robot_leader._run_step()
        r2 = robot_2._run_step()
        r3 = robot_1._run_step()
        if (r2 and r3):
            break
        # if (r1 and r2 and r3):
        #    break

    # robot_leader._stop()
    # robot_follower._stop()
    robot_2._stop()
    robot_1._stop()


if __name__ == '__main__':
    main()
