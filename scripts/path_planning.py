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
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from trabajo_planificacion.srv import *

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import atan2,degrees


voro_publisher = rospy.Publisher('visualization_voronoi', Marker, queue_size=10)
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
        self.stay_nodes =[]
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
        #print self.voro.vertices[1]
        #print self.voro.vertices[3]
        #print self.voro.vertices[21]

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
                self.stay_nodes.append(edge[0])
                self.stay_nodes.append(edge[1])
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

        #print self.graph
        #print self.voro.vertices[1]
        # Para saber que punto 2D se corresponde con cada nodo:
        # Por ejemplo para el nodo 1:
        # print self.voro.vertices[1]
        #voronoi_plot_2d(self.voro)
        #plt.show()




    def checkNeighbors(self,edge):
        n1 = self.voro.vertices[edge[0]]
        n2 = self.voro.vertices[edge[1]]
        if self.map[int(n1[0])][int(n1[1])]:
            return True
        if self.map[int(n2[0])][int(n2[1])]:
            return True
        return False


    def a_star_path(self,start,goal):
        print "start:", start
        print "goal: ", goal

        openSet = set()
        closedSet = set()
        openSet.add(start)
        closedSet.add(start)
        costSoFar = {}
        cameFrom = {}
        cameFrom[start] = None
        for n in self.stay_nodes:
            costSoFar[n] = 10000000
        costSoFar[start] = 0
        print costSoFar
        while len(openSet):
            current = min(costSoFar, key=costSoFar.get)
            if current == goal:
                break
            print current
            openSet.remove(current)
            closedSet.add(current)
            for neighbour in self.graph[current]:
                print neighbour
                if neighbour in closedSet:
                    continue
                else:
                    openSet.add(neighbour)
                    new_cost = costSoFar[current] + self.cost(current, neighbour)
                    print new_cost
                    if new_cost < costSoFar[neighbour]:
                        costSoFar[neighbour] = new_cost
                        cameFrom[neighbour] = current
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
    def plot(self,o_path,f_path,path):
        voronoi_plot_2d(self.voro)
        ax = plt.axes()
        v1 = self.voro.vertices[path[0]]
        plt.plot([o_path[0],v1[0]],[o_path[1],v1[1]])
        for i in range(len(path)-1):
            v1 = self.voro.vertices[path[i]]
            v2 = self.voro.vertices[path[i+1]]
            plt.plot([v1[0],v2[0]],[v1[1],v2[1]],'r')
        plt.show()


    def compute_path(self,req):
        print req.start
        print req.final
        o_path = [req.start.pose.position.x, req.start.pose.position.y]
        f_path = [req.final.pose.position.x, req.final.pose.position.y]
        res = path_calcResponse()
        #print o_path
        first=True
        nodo_i = []
        nodo_f = []
        res.result = 0
        #res.path.header.stamp = Time.now()
        #res.path.header.frame_id = "/map";
        pose = Pose()
        pose.position.x = req.start.pose.position.x
        pose.position.y = req.start.pose.position.y
        poseArray = PoseArray()
        poseArray.poses.append(pose)
        res.path = poseArray

        if self.map[int(o_path[0])][int(o_path[1])]==1:
            res.result = 1
            return res
        elif self.map[int(f_path[0])][int(f_path[1])]==1:
            res.result = 2
            return res
        for n in self.stay_nodes:
            v1 = map.voro.vertices[n]
            if first:
                d1 = (v1[0]-o_path[0])*(v1[0]-o_path[0]) + (v1[1]-o_path[1])*(v1[1]-o_path[1])
                d2 = (v1[0]-f_path[0])*(v1[0]-f_path[0]) + (v1[1]-f_path[1])*(v1[1]-f_path[1])
                first = False
                node_i = n
                node_f = n
            else:
                aux = (v1[0]-o_path[0])*(v1[0]-o_path[0]) + (v1[1]-o_path[1])*(v1[1]-o_path[1])
                if aux < d1:
                    d1 = aux
                    node_i = n
                aux = (v1[0] - f_path[0]) * (v1[0] - f_path[0]) + (v1[1] - f_path[1]) * (v1[1] - f_path[1])
                if aux < d2:
                    d2 = aux
                    node_f = n

        path = self.a_star_path(node_i,node_f)
        #Miro los dos primeros nodos para filtrar el nodo 1 por si lo aleja de la ruta.
        v1 = self.voro.vertices[path[0]]
        v2 = self.voro.vertices[path[1]]
        print "v1:",v1
        print "v2:",v2
        print "o_path-v1",o_path[0]-v1[0]
        print "o_path-v1", o_path[1] - v1[1]
        print "o_path-v2", o_path[0] - v2[0]
        print "o_path-v2", o_path[1] - v2[1]
        if o_path[0]-v1[0]>0 and o_path[1]-v1[1]<0 and o_path[0]-v2[0]>0 and o_path[1]-v2[1]<0:
            path.remove(path[0])
        elif o_path[0]-v1[0]>0 and o_path[1]-v1[1]>0 and o_path[0]-v2[0]>0 and o_path[1]-v2[1]>0:
            path.remove(path[0])
        elif o_path[0]-v1[0]<0 and o_path[1]-v1[1]<0 and o_path[0]-v2[0]<0 and o_path[1]-v2[1]<0:
            path.remove(path[0])
        elif o_path[0]-v1[0]<0 and o_path[1]-v1[1]>0 and o_path[0]-v2[0]<0 and o_path[1]-v2[1]>0:
            path.remove(path[0])
        elif o_path[0] - v1[0] < 0 and o_path[1] - v1[1] > 0 and o_path[0] - v2[0] > 0 and o_path[1] - v2[1] < 0:
            path.remove(path[0])
        elif o_path[0] == v1[0]:
            path.remove(path[0])
        for p in path:
            v = self.voro.vertices[p]
            pose = Pose()
            pose.position.x = v[0]
            pose.position.y = v[1]
            poseArray.poses.append(pose)
        pose = Pose()
        pose.position.x = req.final.pose.position.x
        pose.position.y = req.final.pose.position.y
        poseArray.poses.append(pose)
        res.path = poseArray
        #self.plot(o_path,f_path,path)
        return res

    def visualize_voronoi(self):

        print "aqui toy"
        marker_voro_edges = Marker(id=0, header=Header(frame_id='map', stamp=rospy.Time.now()))
        marker_voro_edges.type = Marker.LINE_LIST
        marker_voro_edges.action = Marker.ADD

        marker_voro_edges.scale.x = 0.1

        # marker color
        line_color = ColorRGBA()
        line_color.a = 0.5
        line_color.r = 1.0

        list_v = []
        for edge in self.voro.ridge_vertices:
            p1 = Point()
            aux = self.voro.vertices[edge[0]]
            p1.x = aux[0]
            p1.y = aux[1]
            p1.z = 0.0
            # print  p.x
            # print  p.y
            # print  p.z
            # print p1
            marker_voro_edges.points.append(p1)
            marker_voro_edges.colors.append(line_color)
            aux = self.voro.vertices[edge[1]]
            p2 = Point()
            p2.x = aux[0]
            p2.y = aux[1]
            p2.z = 0.0
            # print  p.x
            # print  p.y
            # print  p.z
            # print p2
            marker_voro_edges.points.append(p2)
            marker_voro_edges.colors.append(line_color)
            voro_publisher.publish(marker_voro_edges)


if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    input = raw_input("Please enter map path: ")
    map = Map(input)
    map.storeMap()
    map.readMap()
    map.createVoronoi()
    map.visualize_voronoi()
    rospy.Service('path_calc',path_calc,map.compute_path)
    while not rospy.is_shutdown():
        rospy.spin()
        rospy.sleep(1)






