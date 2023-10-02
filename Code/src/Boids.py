import numpy as np
from src.RobotMQ import *


class Boids(RobotMQ):
    def __init__(self, robotname, maxv):
        RobotMQ.__init__(self, robotname, maxv=maxv)
        _, self.robotPos = sim.simxGetObjectPosition(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        _, self.robotOri = sim.simxGetObjectOrientation(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)

    def _boids_step(self, neighbors):
        # Constantes para as regras de alinhamento, coesão e separação
        ka = 0.1
        kr = 0.2
        ks = 0.3

        # Alinhamento: Calcule a média das orientações dos vizinhos
        avg_theta = np.mean([neighbor.robotOri[2] for neighbor in neighbors])
        # Calcule o erro de orientação
        theta_err = avg_theta - self.robotOri[2]
        # Use o erro de orientação para calcular a velocidade angular
        w = ka * theta_err

        # Coesão: Calcule o centroide dos vizinhos
        centroid = np.mean([neighbor.robotPos for neighbor in neighbors], axis=0)
        # Calcule o vetor para o centroide
        vector_to_centroid = centroid - self.robotPos
        # Inicialize a velocidade linear v aqui
        v = np.zeros_like(vector_to_centroid)
        # Adicione este vetor à velocidade linear
        v += kr * vector_to_centroid

        # Separação: Para cada vizinho, calcule um vetor apontando para longe do vizinho
        for neighbor in neighbors:
            vector_away_from_neighbor = np.array(self.robotPos) - np.array(neighbor.robotPos)
            # Adicione este vetor à velocidade linear
            v += ks * vector_away_from_neighbor

        # Limitar v e w para +/- max
        v = np.clip(v, -self.maxv, self.maxv)
        w = max(min(w, self.maxw), -self.maxw)

        # Verifique se v e w são escalares e, se não forem, pegue o primeiro elemento
        if isinstance(v, np.ndarray) and v.size > 1:
            v = v[0]
        if isinstance(w, np.ndarray) and w.size > 1:
            w = w[0]

        # Cinemática Inversa
        wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
        wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)

        # Converter wr e wl para floats se eles forem arrays NumPy
        if isinstance(wr, np.ndarray):
            wr = wr.item()
        if isinstance(wl, np.ndarray):
            wl = wl.item()

        # Enviar velocidades para as rodas
        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)

        # Enviar velocidades para as rodas
        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)

    def _follow_step(self, leader):
        ka = 8 / 20
        err_threshold = 1.2
      
        # Obter a posição e orientação do líder
        _, leaderPos = sim.simxGetObjectPosition(
            self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        _, leaderOri = sim.simxGetObjectOrientation(
            self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        leaderConfig = np.array([leaderPos[0], leaderPos[1], leaderOri[2]])

        # Obter a posição e orientação do seguidor
        _, followerPos = sim.simxGetObjectPosition(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        _, followerOri = sim.simxGetObjectOrientation(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        robotConfig = np.array([followerPos[0], followerPos[1], followerOri[2]])

        # Calcule a diferença de orientação
        dOri = leaderOri[2] - followerOri[2]

        dx, dy, _ = leaderConfig - robotConfig
        err = np.sqrt(dx**2 + dy**2)

        # Defina uma velocidade angular proporcional à diferença de orientação
        w = ka * dOri

        # Defina uma velocidade constante para o seguidor
        v = self.maxv

        # Cinemática Inversa
        wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
        wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)

        # Enviar velocidades para as rodas
        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)
        
        if err < err_threshold:
            self.maxv = .1