#!/usr/bin/python
import rospy
import csv
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import scipy as sp
import time

import matplotlib.pyplot as plt



#Store csvfile data in a map

def timing(f):
    def wrap(*args):
        time1 = time.time()
        ret = f(*args)
        time2 = time.time()
        print '%s function took %0.3f ms' % (f.func_name, (time2-time1)*1000.0)
        return ret
    return wrap

class Map:
    def __init__(self, csvfile):
        self.map=[]
        self.csvfile = csvfile
        self.points = []
        self.voro = []
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

        self.voro.ridge_points = np.array(self.voro.ridge_points)
        self.voro.ridge_vertices = np.array(self.voro.ridge_vertices)
        mask = sp.any(self.voro.ridge_vertices == -1, axis=1)  # Identify edges at infinity
        self.voro.ridge_vertices = self.voro.ridge_vertices[~mask]
        self.voro.ridge_points = self.voro.ridge_points[~mask]

        count = 0
        aux = []
        aux2= []
        for edge in self.voro.ridge_vertices:
            if self.checkNeighbors(edge) is False:
                aux.append([edge[0],edge[1]])
                aux2.append(self.voro.ridge_points[count])
            count = count +1

        #print aux
        #print aux2
        self.voro.ridge_points = np.array(aux2)
        self.voro.ridge_vertices = np.array(aux)
        #print self.voro.ridge_vertices
        #print self.voro.vertices
        #voronoi_plot_2d(self.voro)
        #plt.show()

        return True


    def checkNeighbors(self,edge):
        n1 = self.voro.vertices[edge[0]]
        n2 = self.voro.vertices[edge[1]]
        if self.map[int(n1[0])][int(n1[1])]:
            return True
        if self.map[int(n2[0])][int(n2[1])]:
            return True
        return False



if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    map = Map('../maps/map3.csv')
    map.storeMap()
    map.readMap()
    map.createVoronoi()
