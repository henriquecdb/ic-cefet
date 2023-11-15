import numpy as np
import matplotlib.pyplot as plt
from src.RobotMQ import *


class BoidsRobot(RobotMQ):
    boids = []

    def __init__(self, robotname='Pioneer_p3dx', L=0.331, r=0.09751, maxv=1, maxw=np.deg2rad(45), leader=False):
        super().__init__(robotname, L, r, maxv, maxw, leader)

        _, self.robotPos = sim.simxGetObjectPosition(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)

        self.qgoal = np.zeros(2)
        self.position = np.array([self.robotPos[0], self.robotPos[1]])
        self.boundary = np.array([25.0, 25.0])
        self.velocity = np.zeros(2)

        # Valores sugeridos
        self.turnfactor = .5  # 0.2
        self.visualRange = 10
        self.protectedRange = 3
        self.centeringfactor = 1  # 0.0005
        self.avoidfactor = 2  # 0.05
        self.matchingfactor = 1  # 0.05

        # Adicione o robô à lista de "boids" da simulação
        BoidsRobot.add_boid(self)

    def add_boid(boid):
        BoidsRobot.boids.append(boid)

    def neighboring_boids(self):
        # Conta o número de boids vizinhos dentro do alcance visível
        neighboring_boids = 0
        for boid in BoidsRobot.boids:
            if np.linalg.norm(boid.position - self.position) < self.visualRange:
                neighboring_boids += 1
        return neighboring_boids

    def alignment(self):
        # Alinhamento: Steer towards the average heading of local flockmates
        avg_heading = np.mean([boid.velocity for boid in BoidsRobot.boids])
        alignment_force = self.turnfactor * \
            (avg_heading - self.velocity)
        return alignment_force

    def cohesion(self):
        # Coesão: Steer to move toward the average position of local flockmates
        avg_position = np.mean([boid.position for boid in BoidsRobot.boids])
        cohesion_force = self.centeringfactor * (avg_position - self.position)
        return cohesion_force

    def separation(self):
        # Separação: Steer to avoid crowding local flockmates
        separation_force = np.zeros(2)
        for boid in BoidsRobot.boids:
            if np.linalg.norm(boid.position - self.position) < self.protectedRange:
                separation_force += self.protectedRange - \
                    (self.position - boid.position)
        separation_force *= self.avoidfactor
        return separation_force

    def goal_force(self):
        # Força do objetivo: Steer to move toward the goal
        goal_force = self.qgoal[:2] - self.position[:2]
        return goal_force

    def boundary_force(self):
        # Força da borda: Steer to move away from the boundary
        boundary_force = np.zeros(2)

        left_margin = -self.boundary[0]/2
        right_margin = self.boundary[0]/2
        top_margin = -self.boundary[1]/2
        bottom_margin = self.boundary[1]/2

        if self.position[0] < left_margin:
            boundary_force[0] += self.turnfactor
        if self.position[0] > right_margin:
            boundary_force[0] -= self.turnfactor
        if self.position[1] > bottom_margin:
            boundary_force[1] -= self.turnfactor
        if self.position[1] < top_margin:
            boundary_force[1] += self.turnfactor

        return boundary_force

    def _run_step(self):
        alignment_force = self.alignment()
        cohesion_force = self.cohesion()
        separation_force = self.separation()
        boundary_force = self.boundary_force()

        # Atualize a velocidade e a posição do robô com base nas forças calculadas
        total_force = alignment_force + cohesion_force + separation_force + boundary_force

        # Limita a magnitude da total_force a um valor máximo
        max_force_magnitude = 10.0
        if np.linalg.norm(total_force) > max_force_magnitude:
            total_force = (
                total_force / np.linalg.norm(total_force)) * max_force_magnitude

        print(f"Alignment: {alignment_force}")
        print(f"Cohesion: {cohesion_force}")
        print(f"Separation: {separation_force}")
        print(f"Boundary: {boundary_force}")
        print(f"Neighbour: {self.neighboring_boids()}")
        print()

        kr = 0.5  # 6 / 20

        # Agora execute o código da superclasse
        v = kr * np.hypot(total_force[0], total_force[1])
        w = np.arctan2(total_force[1], total_force[0])

        self.velocity = np.array([kr * total_force[0], kr * total_force[1]])

        # Limit v,w to +/- max
        v = max(min(v, self.maxv), -self.maxv)
        w = max(min(w, self.maxw), -self.maxw)

        # Cinemática Inversa
        wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
        wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)

        # Enviando velocidades
        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)
