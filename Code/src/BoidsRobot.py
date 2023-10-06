import numpy as np
from src.RobotMQ import *


class BoidsRobot(RobotMQ):
    boids = []
    def __init__(self, robotname='Pioneer_p3dx', L=0.331, r=0.09751, maxv=1, maxw=np.deg2rad(45), leader=False):
        super().__init__(robotname, L, r, maxv, maxw, leader)

        _, robotPos = sim.simxGetObjectPosition(
        self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)

        self.qgoal = np.zeros(2)
        self.velocity = np.zeros(2)
        self.position = np.array([robotPos[0], robotPos[1]])
        self.boundary = np.array([25.0, 25.0])

        # Valores sugeridos
        self.turnfactor = .5 # 0.2
        self.visualRange = 10
        self.protectedRange = 4
        self.centeringfactor = 0.5 # 0.0005
        self.avoidfactor = .5 # 0.05
        self.matchingfactor = .5 # 0.05
        self.maxspeed = 6
        self.minspeed = 3

        # Adicione o robô à lista de "boids" da simulação
        BoidsRobot.add_boid(self)

    def add_boid(boid):
        BoidsRobot.boids.append(boid)

    def alignment(self):
        # Alinhamento: Steer towards the average heading of local flockmates
        avg_heading = np.mean([boid.velocity for boid in BoidsRobot.boids])
        alignment_force = self.turnfactor * (avg_heading - self.velocity)
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
                separation_force += self.protectedRange - (self.position - boid.position)
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
            total_force = (total_force / np.linalg.norm(total_force)) * max_force_magnitude

        print(alignment_force, cohesion_force, separation_force, boundary_force)

        # Atualiza a velocidade com base na total force e limita-a entre minspeed e maxspeed
        speed = np.linalg.norm(self.velocity + total_force)
        speed = min(max(speed, self.minspeed), self.maxspeed)

        direction = (self.velocity + total_force) / np.linalg.norm(self.velocity + total_force)
        self.velocity = direction * speed

        # Atualiza a posição com base na velocidade atualizada
        self.position += self.velocity  

        # Agora execute o código da superclasse
        v = np.linalg.norm(self.velocity)  
        w = np.arctan2(self.velocity[1], self.velocity[0])  

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

    def _run_step2(self):
        for boid in self.boids:
            xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids, close_dx, close_dy = 0, 0, 0, 0, 0, 0, 0

            for otherboid in self.boids:
                if otherboid != boid:
                    dx = boid.position[0] - otherboid.position[0]
                    dy = boid.position[1] - otherboid.position[1]

                    if abs(dx) < self.visualRange and abs(dy) < self.visualRange:
                        squared_distance = dx*dx + dy*dy

                        if squared_distance < self.protectedRange*self.protectedRange:
                            close_dx += boid.position[0] - otherboid.position[0]
                            close_dy += boid.position[1] - otherboid.position[1]

                        elif squared_distance < self.visualRange*self.visualRange:
                            xpos_avg += otherboid.position[0]
                            ypos_avg += otherboid.position[1]
                            xvel_avg += otherboid.velocity[0]
                            yvel_avg += otherboid.velocity[1]

                            neighboring_boids += 1

            if neighboring_boids > 0:
                xpos_avg /= neighboring_boids
                ypos_avg /= neighboring_boids
                xvel_avg /= neighboring_boids
                yvel_avg /= neighboring_boids

                boid.velocity[0] += (xpos_avg - boid.position[0])*self.centeringfactor + (xvel_avg - boid.velocity[0])*self.matchingfactor
                boid.velocity[1] += (ypos_avg - boid.position[1])*self.centeringfactor + (yvel_avg - boid.velocity[1])*self.matchingfactor

            boid.velocity[0] += close_dx*self.avoidfactor
            boid.velocity[1] += close_dy*self.avoidfactor

            if boid.position[1] > self.boundary[1]:
                boid.velocity[1] -= self.turnfactor
            if boid.position[0] > self.boundary[0]:
                boid.velocity[0] -= self.turnfactor
            if boid.position[0] < -self.boundary[0]:
                boid.velocity[0] += self.turnfactor
            if boid.position[1] < -self.boundary[1]:
                boid.velocity[1] += self.turnfactor

            speed = np.sqrt(boid.velocity[0]*boid.velocity[0] + boid.velocity[1]*boid.velocity[1])

            if speed < self.minspeed:
                boid.velocity *= self.minspeed / speed
            elif speed > self.maxspeed:
                boid.velocity *= self.maxspeed / speed

            boid.position += boid.velocity

        # Agora execute o código da superclasse para cada robô
        for each_b in self.boids:
            v = np.linalg.norm(each_b.velocity)  
            w = np.arctan2(each_b.velocity[1], each_b.velocity[0])  

            # Limit v,w to +/- max
            v = max(min(v, each_b.maxv), -each_b.maxv)
            w = max(min(w, each_b.maxw), -each_b.maxw)

            # Cinemática Inversa
            wr = ((2.0 * v) + (w * each_b.L)) / (2.0 * each_b.r)
            wl = ((2.0 * v) - (w * each_b.L)) / (2.0 * each_b.r)

            # Enviando velocidades
            sim.simxSetJointTargetVelocity(
                each_b.client_id, each_b.l_wheel, wl, sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(
                each_b.client_id, each_b.r_wheel, wr, sim.simx_opmode_oneshot_wait)
