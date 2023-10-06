#!/usr/bin/env python
from src.BoidsRobot import *
from coppeliasim_zmqremoteapi_client import *


def main():
    # Crie instâncias da classe BoidsRobot para cada robô
    robots_qtty = 3
    goal = np.array([2.0, 3.0])
    robots = []
    for i in range(robots_qtty):
        name = 'Pioneer_p3dx_' + str(i+1)
        robots.append(BoidsRobot(robotname=name))
        print(name)
        # Defina o mesmo objetivo para cada robô
        robots[i].qgoal = goal

    # Execute o algoritmo Boids para cada robô
    while True:
        if all(np.linalg.norm(robot.position - goal) < 0.01 for robot in robots):
            break   # Pare o loop se todos os robôs estiverem perto do objetivo

        for i in range(robots_qtty):
            robots[i]._run_step()


if __name__ == "__main__":
    main()
