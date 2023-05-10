import numpy as np
from bugs.Robot import *


class PotentialFields(Robot):
    def __init__(self, x, y, th):
        Robot.__init__(self)
        self._init_values(x, y, th)

    def _run(self):
        err = np.inf
        while err > .05:
            returnCode, robotPos = sim.simxGetObjectPosition(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy, dx))

            kr = 4 / 20
            ka = 8 / 20
            kb = -1.5 / 20

            v = self.att_force(err=err, katt=kr)
            w = ka * alpha + kb * beta

            v = max(min(v, self.maxv), -self.maxv)
            w = max(min(w, self.maxw), -self.maxw)

            wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
            wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)

            sim.simxSetJointTargetVelocity(
                self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(
                self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)

        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)

    def calc_potential_field(self):
        pass

    def att_force(self, err, katt=.01):
        return 0.5 * katt * err

    def att_force2(self, q, goal, k=2):
        f = k * (goal - q)
        f[:, 0] = 1
        return f

    def rep_force(self, q, obs, R=3, krep=.1):
        # Obstacle: (x, y, r)
        v = q - obs[0:2]
        d = np.linalg.norm(v, axis=1) - obs[2]
        d = d.reshape((len(v), 1))

        rep = (1/d**2)*((1/R))*(v/d)

        invalid = np.squeeze(d > R)
        rep[invalid, :] = 0

        return krep * rep
    
    def calc_repulsive_potential(x, y, ox, oy, rr):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * 100.0 * (1.0 / dq - 1.0 / rr) ** 2 # ETA is repulsive potential gain ETA = 100.0
        else:
            return 0.0
    
