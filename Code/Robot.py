try:
    import sim
except:
    print('"sim.py" could not be imported')

import numpy as np


class Robot:

    def __init__(self, robotname='Pioneer_p3dx', L=0.331, r=0.09751, maxv=1.0, maxw=np.deg2rad(45), following=False):
        self.robotname = robotname

        # Específico do robô
        self.L = L
        self.r = r
        self.maxv = maxv
        self.maxw = maxw
        self.following = following

        self._init_client_id()
        self._init_handles()
        self._init_sensors_handle()

    def _init_client_id(self):
        sim.simxFinish(-1)
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:
            print('Connected to remote API server')
        else:
            print('Failed connecting to remote API server')

    def _init_handles(self):
        self._init_robot_handle()
        self._init_target_handle()
        self._init_wheels_handle()

    def _init_robot_handle(self):
        returnCode, self.robotHandle = sim.simxGetObjectHandle(
            self.client_id, self.robotname, sim.simx_opmode_oneshot_wait)

    def _init_target_handle(self):
        returnCode, goalFrame = sim.simxGetObjectHandle(
            self.client_id, 'Goal', sim.simx_opmode_oneshot_wait)

    def _init_wheels_handle(self):
        returnCode, self.l_wheel = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
        returnCode, self.r_wheel = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    def _init_sensors_handle(self):
        returnCode, sonar_front = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_ultrasonicSensor5', sim.simx_opmode_oneshot_wait)
        returnCode, sonar_right = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_ultrasonicSensor7', sim.simx_opmode_oneshot_wait)

    def _init_values(self, x, y, th):
        qgoal = np.array([x, y, np.deg2rad(th)])
        returnCode, goalFrame = sim.simxGetObjectHandle(
            self.client_id, 'Goal', sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectPosition(
            self.client_id, goalFrame, -1, [qgoal[0], qgoal[1], 0], sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectOrientation(
            self.client_id, goalFrame, -1, [0, 0, qgoal[2]], sim.simx_opmode_oneshot_wait)

    def _read_values(self):
        pass
