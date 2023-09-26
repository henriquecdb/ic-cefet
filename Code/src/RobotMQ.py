try:
    import sim
except:
    print('"sim.py" could not be imported')

import numpy as np
from libs.utils import *
from coppeliasim_zmqremoteapi_client import *


class RobotMQ:
    def __init__(self, robotname='Pioneer_p3dx', L=0.331, r=0.09751, maxv=1.0, maxw=np.deg2rad(45), leader=False):
        self.robotname = robotname
        self.L = L
        self.r = r
        self.maxv = maxv
        self.maxw = maxw
        self.leader = leader

        self._init_client_id()
        self._init_handles()

    def _init_client_id(self):
        sim.simxFinish(-1)
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:
            print('Connected to remote API server')
        else:
            print('Failed connecting to remote API server')

    def _init_handles(self):
        self._init_robot_handle()
        self._init_wheels_handle()

    def _init_robot_handle(self):
        client = RemoteAPIClient('localhost', 23000)
        sim = client.getObject('sim')
        var = "/" + self.robotname
        self.robotHandle = sim.getObject(var)

    def _init_wheels_handle(self):
        client = RemoteAPIClient('localhost', 23000)
        sim = client.getObject('sim')
        l_wheel = "/" + self.robotname + "_leftMotor"
        r_wheel = "/" + self.robotname + "_rightMotor"
        self.l_wheel = sim.getObject(l_wheel)
        self.r_wheel = sim.getObject(r_wheel)

    def run(self):
        err = np.inf
        it = 0
        while err > .05:
            # Pos, Ori
            _, robotPos = sim.simxGetObjectPosition(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            _, robotOri = sim.simxGetObjectOrientation(
                self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy, dx))

            kr = 4 / 20
            ka = 8 / 20
            kb = -1.5 / 20

            # if abs(alpha) > np.pi/2:
            # kr = -kr
            # Se não ajustar a direção muda
            #    alpha = normalizeAngle(alpha - np.pi)
            #    beta = normalizeAngle(beta - np.pi)

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
            it += 1

        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)

    def _run_step(self):
        err = np.inf
        # Pos, Ori
        _, robotPos = sim.simxGetObjectPosition(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        _, robotOri = sim.simxGetObjectOrientation(
            self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
        robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

        dx, dy, dth = self.qgoal - robotConfig
        err = np.sqrt(dx**2 + dy**2)
        alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
        beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy, dx))

        # print(robotConfig)
        print(err)

        kr = 4 / 20
        ka = 8 / 20
        kb = -1.5 / 20

        # if abs(alpha) > np.pi/2:
        # kr = -kr
        # Se não ajustar a direção muda
        #    alpha = normalizeAngle(alpha - np.pi)
        #    beta = normalizeAngle(beta - np.pi)

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

        if (err < .2):
            return True

    def _stop(self):
        sim.simxSetJointTargetVelocity(
            self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)
