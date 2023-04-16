try:
    import sim
except:
    print('"sim.py" could not be imported')

from bugs.Robot import *
import numpy as np
from libs.utils import *


class Bug(Robot):
    
    def __init__(self, x, y, th):
        Robot.__init__(self)
        self._init_values(x, y, th)

    def _init_values(self, x, y, th):
        self.qgoal = np.array([x, y, np.deg2rad(th)])
        returnCode, self.goalFrame = sim.simxGetObjectHandle(
            self.client_id, 'Goal', sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectPosition(
            self.client_id, self.goalFrame, -1, [self.qgoal[0], self.qgoal[1], 0], sim.simx_opmode_oneshot_wait)
        returnCode = sim.simxSetObjectOrientation(
            self.client_id, self.goalFrame, -1, [0, 0, self.qgoal[2]], sim.simx_opmode_oneshot_wait)

    def _bug1(self):
        following = False
        err = np.inf
        start = np.zeros(2)
        it = 0
        while err > .05:
            # Pos, Ori
            returnCode, robotPos = sim.simxGetObjectPosition(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)  
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy,dx))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy,dx))

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
                start = robotPos[:2]
                v = 0
                w = np.deg2rad(30)
                following = True
            else: 
                if obstacle_in_right:
                    w = np.deg2rad(10)
                elif start == robotPos[:2]: # to-do
                    break
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
            #print(following)
            # print(robotPos)
            # print(robotOri)
        
        sim.simxSetJointTargetVelocity(self.client_id, self.l_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(self.client_id, self.r_wheel, 0, sim.simx_opmode_oneshot_wait)

    def _bug2(self):
        following = False
        err = np.inf
        it = 0
        while err > .05:
            # Pos, Ori
            returnCode, robotPos = sim.simxGetObjectPosition(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)
            returnCode, robotOri = sim.simxGetObjectOrientation(self.client_id, self.robotHandle, -1, sim.simx_opmode_oneshot_wait)  
            robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

            dx, dy, dth = self.qgoal - robotConfig
            err = np.sqrt(dx**2 + dy**2)
            alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy,dx))
            beta = normalizeAngle(self.qgoal[2] - np.arctan2(dy,dx))

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
                elif collinear(-2.96, -2.5, robotConfig[0], robotConfig[1], self.qgoal[0], self.qgoal[1]):
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
