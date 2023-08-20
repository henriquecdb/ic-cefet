import numpy as np
import math
from src.Robot import *
from src.PotentialFields import *


class Leader(Robot):
    def __init__(self, x, y, th):
        Robot.__init__(self)
        self._init_values(x, y, th)

    def _follow(self, leader):
        _, leaderPos = sim.simxGetObjectPosition(
            self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        _, leaderOri = sim.simxGetObjectOrientation(
            self.client_id, leader.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        target = np.array([leaderPos[0], leaderPos[1], leaderOri[2]])
        err = np.inf
        while err > .05:
            # Pos, Ori
            returnCode, robotPos = sim.simxGetObjectPosition(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy, dx))

            print(robotConfig)

            kr = 4 / 20
            ka = 8 / 20
            kb = -1.5 / 20

            v = kr * err
            w = ka * alpha + kb * beta

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

        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)
