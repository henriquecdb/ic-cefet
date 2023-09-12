def calc_potential_field(self, gx, gy, ox, oy, reso, rr, sx, sy):
        minx = min(min(ox), sx, gx) - self.AREA_WIDTH / 2.0
        miny = min(min(oy), sy, gy) - self.AREA_WIDTH / 2.0
        maxx = max(max(ox), sx, gx) + self.AREA_WIDTH / 2.0
        maxy = max(max(oy), sy, gy) + self.AREA_WIDTH / 2.0
        xw = int(round((maxx - minx) / reso))
        yw = int(round((maxy - miny) / reso))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * reso + minx
            for iy in range(yw):
                y = iy * reso + miny
                ug = 0.5 * 5 * np.hypot(x - gx, y - gy) # attractive force
                uo = self.calc_repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
                pmap[ix][iy] = uf
        return pmap, minx, miny

def att_force2(self, q, goal, k=2):
        f = k * (goal - q)
        f[:, 0] = 1
        return f

def rep_force2(self, q, obs, R=3, krep=.1):
        # Obstacle: (x, y, r)
        v = q[:2] - obs[0:2]
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
        