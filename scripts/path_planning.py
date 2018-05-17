#!/usr/bin/python
import rospy
import csv
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import scipy as sp
import time

import matplotlib.pyplot as plt
from collections import defaultdict


from geometry_msgs.msg import PoseStamped
from trabajo_planificacion.srv import *
path = []
def timing(f):
    def wrap(*args):
        time1 = time.time()
        ret = f(*args)
        time2 = time.time()
        print '%s function took %0.3f ms' % (f.func_name, (time2-time1)*1000.0)
        return ret
    return wrap

#Store csvfile data in a map
class Map:
    def __init__(self, csvfile):
        self.map=[]
        self.csvfile = csvfile
        self.points = []
        self.voro = []
        self.graph = {}
    @timing
    def storeMap(self):
        self.map=[]
        with open(self.csvfile, 'rb') as csvfile:
            self.map = np.loadtxt(self.csvfile,dtype=int, delimiter=",")

    @timing
    def readMap(self):
        for row in self.map:
            print row

    @timing
    def createVoronoi(self):
        for i in range(0,len(self.map)):
            for j in range(0,len(self.map[0])):
                if self.map[i][j] == 1:
                    self.points.append([i,j])
                    #if j+1<len(self.map[0]):
                    self.points.append([i,j+1])
                    self.points.append([i+1,j+1])
                    #        if i+1<len(self.map):
                    self.points.append([i + 1, j])
        self.points = np.array(self.points)
        self.voro = Voronoi(self.points)
        #print self.voro.point_region
        #Filtrate voronoi
        # print vor.ridge_points

        #Delete edges to infinity
        print self.voro.vertices[1]
        print self.voro.vertices[3]
        print self.voro.vertices[21]

        self.voro.ridge_points = np.array(self.voro.ridge_points)
        self.voro.ridge_vertices = np.array(self.voro.ridge_vertices)
        mask = sp.any(self.voro.ridge_vertices == -1, axis=1)  # Identify edges at infinity
        self.voro.ridge_vertices = self.voro.ridge_vertices[~mask]
        self.voro.ridge_points = self.voro.ridge_points[~mask]

        count = 0
        aux = []
        aux2= []
        # check if node is between four ridge points
        for edge in self.voro.ridge_vertices:
            if self.checkNeighbors(edge) is False:
                aux.append([edge[0],edge[1]])
                aux2.append(self.voro.ridge_points[count])
            count = count +1

        #print aux
        #print aux2
        self.voro.ridge_points = np.array(aux2)
        self.voro.ridge_vertices = np.array(aux)
        print self.voro.ridge_vertices
        #print self.voro.vertices
        print self.voro.ridge_vertices


        # Store edges in a dictionary.
        # For each node store the connected nodes


        for r in self.voro.ridge_vertices:
            self.graph.setdefault(r[0], []).append(r[1])
            self.graph.setdefault(r[1], []).append(r[0])

        print self.graph
        print self.voro.vertices[1]
        # Para saber que punto 2D se corresponde con cada nodo:
        # Por ejemplo para el nodo 1:
        # print self.voro.vertices[1]




    def checkNeighbors(self,edge):
        n1 = self.voro.vertices[edge[0]]
        n2 = self.voro.vertices[edge[1]]
        if self.map[int(n1[0])][int(n1[1])]:
            return True
        if self.map[int(n2[0])][int(n2[1])]:
            return True
        return False


    def a_star_path(self,start,goal):
        openSet = set()
        closedSet = set()
        openSet.add(start)
        closedSet.add(start)
        costSoFar = {}
        costSoFar[start] = 0
        cameFrom = {}
        cameFrom[start] = None
        while len(openSet):
            current = min(costSoFar, key=costSoFar.get)
            if current == goal:
                 break
            openSet.remove(current)
            closedSet.add(current)
            first = True
            for next in self.graph[current]: # Neighbors of current
                if first:
                    previous = next
                    first = False
                if next not in closedSet:
                    new_cost = costSoFar[current] + self.cost(current, next)
                    if previous in costSoFar:
                        if new_cost < costSoFar[previous]:
                            costSoFar[next] = new_cost
                            cameFrom[next] = current
                            openSet.add(next)
                            previous = next
                    else:
                        costSoFar[next] = new_cost
                        cameFrom[next] = current
                        openSet.add(next)
                        previous = next
            first = True
            costSoFar.pop(current)
        return self.retracePath(goal, start, cameFrom)

        #             frontier.put(next, new_cost)
        #             came_from[next] = current

    def retracePath(self,goal,start,cameFrom):
        path = [goal]
        c=goal
        while c is not start:
            c = cameFrom[c]
            path.append(c)

        path.reverse()
        return path


    def cost(self,node1,node2):
        p1 = self.voro.vertices[node1]
        p2 = self.voro.vertices[node2]
        return (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1])
    def plot(self,path):
        voronoi_plot_2d(self.voro)
        ax = plt.axes()
        for i in range(len(path)-1):
            v1 = self.voro.vertices[path[i]]
            v2 = self.voro.vertices[path[i+1]]
            plt.plot([v1[0],v2[0]],[v1[1],v2[1]],'r')
        plt.show()


def compute_path(req):
    print req.start
    print req.final
    o_path = [req.start.pose.position.x, req.start.pose.position.y]
    f_path = [req.final.pose.position.x, req.final.pose.position.y]

    print o_path
    first=True
    nodo_i = []
    nodo_f = []
    for edge in map.voro.ridge_vertices:
        v1 = map.voro.vertices[edge[0]]
        if first:
            d1 = (v1[0]-o_path[0])*(v1[0]-o_path[0]) + (v1[1]-o_path[1])*(v1[1]-o_path[1])
            d2 = (v1[0]-f_path[0])*(v1[0]-f_path[0]) + (v1[1]-f_path[1])*(v1[1]-f_path[1])
            first = False
            node_i = edge[0]
            node_f = edge[0]
        else:
            aux = (v1[0]-o_path[0])*(v1[0]-o_path[0]) + (v1[1]-o_path[1])*(v1[1]-o_path[1])
            if aux < d1:
                d1 = aux
                node_i = edge[0]
            aux = (v1[0] - f_path[0]) * (v1[0] - f_path[0]) + (v1[1] - f_path[1]) * (v1[1] - f_path[1])
            if aux < d2:
                d2 = aux
                node_f = edge[0]
    print node_i
    print node_f
    path = map.a_star_path(node_i,node_f)
    map.plot(path)

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    input = raw_input("Please enter map path: ")
    map = Map(input)
    map.storeMap()
    map.readMap()
    map.createVoronoi()
    rospy.Service('path_calc',path_calc,compute_path)
    rospy.spin()






