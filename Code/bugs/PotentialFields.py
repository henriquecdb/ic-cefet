import numpy as np
from Robot import *


class PotentialFields(Robot):
    def __init__(self, robotname='Pioneer_p3dx', L=0.331, r=0.09751, maxv=1, maxw=np.deg2rad(45), following=False, x=0, y=0, th=0):
        super().__init__(robotname, L, r, maxv, maxw, following, x, y, th)

    def _run(self):
        pass

    def att_force(self, q, goal, katt=.01):
        return katt * (goal - q)

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
