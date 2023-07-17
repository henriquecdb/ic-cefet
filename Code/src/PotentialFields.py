import numpy as np
from src.Robot import *

class PotentialFields(Robot):
    def __init__(self, x, y, th):
        Robot.__init__(self)
        self._init_values(x, y, th)
        self.R = 1.25

    def _run(self):
        err = np.inf
        obs = [[0,2,1], [3,-3,1], [-3,-1,1]]
        while err > .05:
            returnCode, robotPos = sim.simxGetObjectPosition(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            #alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
            #beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy, dx))

            #obs = [0, 0, 1]
            theta = robotConfig[2]
            att = self.att_force(robotConfig, self.qgoal, .1)
            #att = [att[0] * cos(theta) + att[1] * -sin(theta), att[0] * sin(theta) + att[1] * cos(theta)]
            #rep = self.rep_force(robotConfig, obs, 2)
            #rep = [rep[0] * cos(theta) + rep[1] * -sin(theta), rep[0] * sin(theta) + rep[1] * cos(theta)]
            rep = self.check_obs(robotConfig, obs)
            result = att + rep

            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(result[1], result[0]))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(result[1], result[0]))
            #beta = normalizeAngle(-robotConfig[2] - alpha)

            kr = 0.5 # 6 / 20
            ka = 12 / 20
            kb = -2.25 / 20

            if abs(alpha) > np.pi/2:
                kr = -kr

            v = kr * np.hypot(result[0], result[1])
            w = ka * alpha + kb * beta

            #print(att[0], att[1])
            #print(rep[0], rep[1])
            print(f'att:{att}, rep:{rep}')
            print(f'att:{np.hypot(att[0], att[1])}, rep:{np.hypot(rep[0], rep[1])}')
            print(f'res:{result}')

            #v = max(min(v, self.maxv), -self.maxv)
            #w = max(min(w, self.maxw), -self.maxw)

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

    def att_force(self, q, goal, katt=.01):
        return katt * (goal[:2] - q[:2])

    def rep_force(self, q, obs, krep=.1):
        p0 = self.R
        pi = np.hypot(q[0] - obs[0], q[1] - obs[1])
        if (pi <= p0 + 1):
            print(f'pi: {pi}, p0+1: {p0+1}')
            #return ((1/pi) - (1/p0)) * krep/(pi**2) * (q[0:2] - obs[0:2])/pi
            return 1/pi * krep/(pi**2) * (q[0:2] - obs[0:2])/pi
        else:
            return [0, 0]
        
    
    def check_obs(self, robotConfig, obs):
        rep_result = [0, 0]
        for i, _ in enumerate(obs):
            if (np.hypot(robotConfig[0] - obs[i][0], robotConfig[1] - obs[i][1]) <= self.R+obs[i][2]):
                aux = [obs[i][0], obs[i][1], obs[i][2]]
                rep_result += self.rep_force(robotConfig, aux, 1)
                #print(rep_result)
        return rep_result
