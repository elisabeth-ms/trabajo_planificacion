#!/usr/bin/python
import rospy
import csv
import yaml
import time
import numpy as np
import string
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
    def __init__(self, csvfile,resolution):
        self.map=[]
        self.csvfile = csvfile
        self.resolution = resolution
        self.map_name = ''
        self.yaml_name = ''
        self.image_name = ''
        self.yaml_image_name = ''
    @timing
    def storeMap(self):
        self.map=[]
        with open(self.csvfile, 'rb') as csvfile:
            self.map = np.loadtxt(self.csvfile,dtype=int, delimiter=",")

    @timing
    def readMap(self):
        for row in self.map:
            print row
    def readString(self):
        str = self.csvfile[::-1]
        self.map_name = ''
        self.map_name = str[4:len(str):1]
        self.yaml_image_name = str[4:8:1]
        self.yaml_image_name = self.yaml_image_name[::-1]
        self.yaml_image_name = self.yaml_image_name + ".pgm"
        self.map_name = self.map_name[::-1]
        self.yaml_name = self.map_name + ".yaml"
        self.image_name = self.map_name + ".pgm"
        print self.yaml_image_name

    def createYaml(self):
        recipe = {
            'name': 'sushi',
            'ingredients': ['rice', 'vinegar', 'sugar', 'salt', 'tuna', 'salmon'],
            'cooking time': 10,
            'difficulty': 'difficult'
        }
        data = {
            'image': self.yaml_image_name,
            'resolution': float(self.resolution),
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'origin': [0.0,0.0,0.0]
        }
        with open(self.yaml_name, 'w') as outfile:
            yaml.dump(data, outfile)

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    input = raw_input("Please enter map path: ")
    resolution = raw_input("Please enter the desired resolution: ")
    map = Map(input,resolution)
    map.storeMap()
    map.readString()
    map.createYaml()
    map.readMap()

    #rospy.spin()






