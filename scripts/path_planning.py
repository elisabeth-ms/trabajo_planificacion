#!/usr/bin/python
import rospy
import csv
import numpy as np

#Store csvfile data in a map
class Map:
    def __init__(self, csvfile):
        self.map=[]
        self.csvfile = csvfile
    def storeMap(self):
        with open(self.csvfile, 'rb') as csvfile:
            self.map = np.loadtxt(self.csvfile,dtype=int, delimiter=",")

    def ReadMap(self):
        for row in self.map:
            print row


if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    map = Map('../maps/map1.csv')
    map.storeMap()
    map.ReadMap()
