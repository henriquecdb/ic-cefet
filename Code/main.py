#!/usr/bin/env python
from src.BoidsRobot import *
from coppeliasim_zmqremoteapi_client import *


def main():
    # Crie instâncias da classe BoidsRobot para cada robô
    robot1 = BoidsRobot(robotname='Pioneer_p3dx_1')
    robot2 = BoidsRobot(robotname='Pioneer_p3dx_2')
    robot3 = BoidsRobot(robotname='Pioneer_p3dx_3')

    # Defina o mesmo objetivo para cada robô
    goal = np.array([2.0, 3.0])
    robot1.qgoal = goal
    robot2.qgoal = goal
    robot3.qgoal = goal

    # Adicione cada robô à lista de "boids" dos outros robôs
    robot1.add_boid(robot2)
    robot1.add_boid(robot3)

    robot2.add_boid(robot1)
    robot2.add_boid(robot3)

    robot3.add_boid(robot1)
    robot3.add_boid(robot2)

    # Execute o algoritmo Boids para cada robô
    while True:
        if np.linalg.norm(robot1.position - goal) < 0.01 and \
           np.linalg.norm(robot2.position - goal) < 0.01 and \
           np.linalg.norm(robot3.position - goal) < 0.01:
            break  # Pare o loop se todos os robôs estiverem perto do objetivo

        robot1._run_step()
        robot2._run_step()
        robot3._run_step()


if __name__ == "__main__":
    main()
