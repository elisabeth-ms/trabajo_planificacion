#!/usr/bin/python
"""

Potential Field based path planner

"""

import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid
from trabajo_planificacion.srv import *
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
rrt_publisher = rospy.Publisher('visualization_rrt', Marker, queue_size=10)
# Parameters
KP = 4.0  # attractive potential gain
AREA_WIDTH = 8.0  # potential area width [m]
d_goal = 6.0
show_animation = True
globalx = 4.5  # goal x position [m]
globaly = 14.5  # goal y position [m]

class PotentialField:
    def __init__(self):
        self.map=[]
        self.res = 1
        self.ox = []
        self.oy = []
        self.repulsive_map=[]
        self.Krep = 50.0  # repulsive potential gain
        self.rr = 2.0 # robot radius [m]

    def callback_map(self,data):
        self.res = data.info.resolution
        self.map= np.zeros((data.info.width,data.info.height),dtype=int)
        self.repulsive_map = np.zeros((data.info.width,data.info.height),dtype=int)
        self.ox = []
        self.oy = []
        self.readMap()
        for i in range(0,data.info.width):
            for j in range(0,data.info.height):
                #print data.data[j * data.info.width + i]
                if data.data[j*data.info.width+i]==100:
                    self.map[i][j]=1
                    self.ox.append(i)
                    self.oy.append(j)
        self.readMap()
        # El campo potencial de repulsion lo calculo cuando recibo los obstaculos
        self.calc_repulsive_potential(self.ox,self.oy,self.rr)


    def readMap(self):
        for row in self.map:
            print row

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
                uo = calc_repulsive_potential(self.ox, self.oy, self.rr)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny


    def calc_attractive_potential(selff,x, y, gx, gy):
        d = np.hypot(x - gx, y - gy)
        if d_goal<= d:
            return 0.5 * KP * d_goal*d_goal
        else:
            return KP*d*d_goal

    def calc_repulsive_potential(self,ox, oy, rr):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for x in range(len(self.repulsive_map)):
            for y in range(len(self.repulsive_map[0])):
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
                    self.repulsive_map[x][y] = 0.5 * self.Krep * (1.0 / dq - 1.0 / rr) ** 2
                else:
                    self.repulsive_map[x][y] = 0.0
        print self.repulsive_map
        return True

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
                         #Nuevo destino local
                        print "nuevo"

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

def main():
    print("Potential_field_planning start")
    rospy.init_node('path_planning_pF', anonymous=True)
    potentialField = PotentialField()
    sx = 10  # start x position [m]
    sy = 14  # start y positon [m]
    grid_size = 1.0  # potential grid size [m]

    rospy.Subscriber("move_base/global_costmap/costmap",OccupancyGrid,potentialField.callback_map)
    #rospy.Service('path_calc_potential_field',path_calc,potentialField.compute_path)

    while not rospy.is_shutdown():
        rospy.spin()
        rospy.sleep(1)
    # path generation
    rx, ry = potential_field_planning(
        sx, sy, globalx, globaly, ox, oy, grid_size, robot_radius)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
