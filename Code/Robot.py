try:
    import sim
except:
    print('"sim.py" could not be imported')

#from ic-cefet.Code.libs.utils import normalizeAngle
import numpy as np
from libs.utils import *

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
        self._init_values(4, 4, 0)

        self.run()

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
        returnCode, self.sonar_front = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_ultrasonicSensor5', sim.simx_opmode_oneshot_wait)
        returnCode, self.sonar_right = sim.simxGetObjectHandle(
            self.client_id, self.robotname + '_ultrasonicSensor7', sim.simx_opmode_oneshot_wait)

    def _init_values(self, x, y, th):
        self.qgoal = np.array([x, y, np.deg2rad(th)])
        returnCode, self.goalFrame = sim.simxGetObjectHandle(
            self.client_id, 'Goal', sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectPosition(
            self.client_id, self.goalFrame, -1, [self.qgoal[0], self.qgoal[1], 0], sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectOrientation(
            self.client_id, self.goalFrame, -1, [0, 0, self.qgoal[2]], sim.simx_opmode_oneshot_wait)

    def _read_values(self):
        pass

    def run(self):
        qgoal = np.array([-4, 4, np.deg2rad(90)])
        following = False
        err = np.inf
        it = 0
        while err > .05:
            # Pos, Ori
            returnCode, robotPos = sim.simxGetObjectPosition(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)  
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy,dx))
            beta = normalizeAngle(qgoal[2] - np.arctan2(dy,dx))

            print(robotConfig)

            # Fazendo leitura dos sensores
            returnCode, detected_front, point_front, *_ = sim.simxReadProximitySensor(self.client_id, self.sonar_front, sim.simx_opmode_oneshot_wait)
            returnCode, detected_right, point_right, *_ = sim.simxReadProximitySensor(self.client_id, self.sonar_right, sim.simx_opmode_oneshot_wait)
            
            kr = 4 / 20
            ka = 8 / 20
            kb = -1.5 / 20

            if abs(alpha) > np.pi/2:
                kr = -kr       
                # Se não ajustar a direção muda
                alpha = normalizeAngle(alpha - np.pi)
                beta = normalizeAngle(beta - np.pi)
            
            v = kr * err
            w = ka * alpha + kb * beta

            # Limit v,w to +/- max
            v = max(min(v, self.maxv), -self.maxv)
            w = max(min(w, self.maxw), -self.maxw) 

            obstacle_in_front = (detected_front and np.linalg.norm(point_front) < .9)
            obstacle_in_right = (detected_right and np.linalg.norm(point_right) < .9)

            # Controle
            if obstacle_in_front:
                v = 0
                w = np.deg2rad(30)
                following = True
            else: 
                if obstacle_in_right:
                    w = np.deg2rad(10)
                elif collinear(-2.96, -2.5, robotConfig[0], robotConfig[1], qgoal[0], qgoal[1]):
                    following = False
                elif following:
                    v = .1
                    w = np.deg2rad(-30)

            # Cinemática Inversa
            wr = ((2.0 * v) + (w * self.L)) / (2.0 * self.r)
            wl = ((2.0 * v) - (w * self.L)) / (2.0 * self.r)    
            
            # Enviando velocidades
            sim.simxSetJointTargetVelocity(self.client_id, self.l_wheel, wl, sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(self.client_id, self.r_wheel, wr, sim.simx_opmode_oneshot_wait)
            it += 1
            print(following)
            # print(robotPos)
            # print(robotOri)
        
        sim.simxSetJointTargetVelocity(self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)