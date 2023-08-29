import numpy as np
from src.Robot import *


class Follower(Robot):
    def __init__(self, leader):
        Robot.__init__(self)
        # self._init_values(x, y, th)
        self._follow(leader)

    def _follow(self, leader):
        err = np.inf
        err_threshold = 0.1
        kr = 4 / 20
        ka = 8 / 20
        kb = -1.5 / 20

        leader.run()

        while True:
            # Obter a posição do líder
            _, leaderPos = sim.simxGetObjectPosition(
                self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            _, leaderOri = sim.simxGetObjectOrientation(
                self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            leaderConfig = np.array([leaderPos[0], leaderPos[1], leaderOri[2]])

            # Obter a posição e orientação do seguidor
            _, robotPos = sim.simxGetObjectPosition(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            _, robotOri = sim.simxGetObjectOrientation(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            print(robotConfig, leaderConfig)

            # Calcule o erro de posição
            dx, dy, dth = leaderConfig - robotConfig
            err = np.sqrt(dx**2 + dy**2)

            # Calcule os ângulos alpha e beta
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
            beta = normalizeAngle(leaderConfig[2] - np.arctan2(dy, dx))

            # Calcule as velocidades v e w usando o controlador de campos potenciais
            v = kr * err
            w = ka * alpha + kb * beta

            # Limitar v e w para +/- max
            v = max(min(v, self.maxv), -self.maxv)
            w = max(min(w, self.maxw), -self.maxw)

            # Cinemática Inversa
            wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
            wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)

            # Enviar velocidades para as rodas
            sim.simxSetJointTargetVelocity(
                self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(
                self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)

            # Verifique se o erro está abaixo do limiar para parar de seguir
            if err < err_threshold:
                break

        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)
