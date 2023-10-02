#!/usr/bin/env python
from src.Boids import *
from coppeliasim_zmqremoteapi_client import *


def main():
    robot_2 = RobotMQ(robotname="Pioneer_p3dx_2", maxv=0.3)
    #qgoals = [[2, 3, 0], [2, -3, 0], [-2, -3, 0], [-2, 3, 0]]
    qgoals = [[2, 3, 0]]

    robot_1 = Boids(robotname="Pioneer_p3dx_1", maxv=0.3)
    robot_3 = Boids(robotname="Pioneer_p3dx_3", maxv=0.3)

    for qgoal in qgoals:
        robot_2.qgoal = qgoal
        while (True):
            r2 = robot_2._run_step()
            robot_1._follow_step(robot_2)
            robot_3._follow_step(robot_2)
            if r2:
                break

    robot_2._stop()
    robot_1._stop()
    robot_3._stop()


if __name__ == '__main__':
    main()
