#!/usr/bin/env python
from src.Follower import *


def main():
    robot_leader = Robot(robotname="Pioneer_p3dx_2", leader=True, maxv=0.3)
    robot_leader.qgoal = [-4, 3, 0]
    robot_follower = Follower(robot_leader)

    while (True):
        r1 = robot_leader._run_step()
        r2 = robot_follower._follow_step(robot_leader)

        if (r1 and r2):
            break
    
    robot_leader._stop()
    robot_follower._stop()

if __name__ == '__main__':
    main()
