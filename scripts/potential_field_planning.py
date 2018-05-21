#!/usr/bin/python
"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 4.0  # attractive potential gain
ETA = 50.0  # repulsive potential gain
AREA_WIDTH = 8.0  # potential area width [m]
d_goal = 6.0
show_animation = True
globalx = 4.5  # goal x position [m]
globaly = 14.5  # goal y position [m]


def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    d = np.hypot(x - gx, y - gy)
    if d_goal<= d:
        return 0.5 * KP * d_goal*d_goal
    else:
        return KP*d*d_goal

def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i in range(len(ox)):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)
    iteraciones = 0
    x_comp = sx
    y_comp = sy
    #print pmap
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
       draw_heatmap(pmap)
       plt.plot(ix, iy, "*k")
       plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    global termine
    while not termine:

        minp = float("inf")
        minix, miniy = -1, -1
        for i in range(len(motion)):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])

            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny

        if xp == gx and yp == gy:
            if gx != globalx and gy != globaly:
                potential_field_planning(gx, gy, globalx, globaly, ox, oy, reso, rr)
            else:
                termine = True
                print "fini"
                return rx,ry
        else:
             if np.hypot(xp-x_comp,yp - y_comp)<1.5:
                 iteraciones = iteraciones + 1
                 if iteraciones>5:
                    print iteraciones
                    potential_field_planning(xp, yp, 7, 23, ox, oy, reso, rr)
             else:
                 iteraciones = 0
                 x_comp = xp
                 y_comp = yp
                 rx.append(x_comp)
                 ry.append(y_comp)

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")
    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=1000.0, cmap=plt.cm.Blues)

class Map:
    def __init__(self, csvfile):
        self.map=[]
        self.csvfile = csvfile


    def storeMap(self):
        self.map=[]
        with open(self.csvfile, 'rb') as csvfile:
            self.map = np.loadtxt(self.csvfile,dtype=int, delimiter=",")

def main():
    print("potential_field_planning start")
    input = raw_input("Please enter map path: ")
    map = Map(input)
    map.storeMap()
    sx = 10  # start x position [m]
    sy = 14  # start y positon [m]
    grid_size = 1.0  # potential grid size [m]
    robot_radius = 2.0 # robot radius [m]
    ox = []
    oy = []
    global termine
    termine = False
    for i in range(len(map.map)):
        for j in range(len(map.map[0])):
           if map.map[i][j] == 1:
                ox.append(i)
                oy.append(j)

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    rx, ry = potential_field_planning(
        sx, sy, globalx, globaly, ox, oy, grid_size, robot_radius)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
