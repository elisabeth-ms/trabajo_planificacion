#!/usr/bin/python
import rospy
import csv
import yaml
import time
import numpy as np
import string
from PIL import Image


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
        self.resolution = float(resolution)
        self.map_name = ''
        self.yaml_name = ''
        self.image_name = ''
        self.yaml_image_name = ''
    @timing
    def storeMap(self):
        self.map=[]
        aux_map =[]
        with open(self.csvfile, 'rb') as csvfile:
            aux_map = np.loadtxt(self.csvfile,dtype=int, delimiter=",")

        new_width = int(len(aux_map[0])/self.resolution)
        new_height = int(len(aux_map)/self.resolution)
        s = (new_height,new_width)
        self.map = np.zeros(s,dtype='int')

        print aux_map
        for i in range(new_height):
            for j in range(new_width):
                if aux_map[int(i*self.resolution), int(j*self.resolution)] == 1:
                    self.map[i,j] = 1
        print self.map
        #self.map = self.map.T
        #print self.map


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
        self.yaml_image_name = self.yaml_image_name + ".png"
        self.map_name = self.map_name[::-1]
        self.yaml_name = self.map_name + ".yaml"
        self.image_name = self.map_name + ".png"
        self.map = np.array(self.map)
        self.map = np.fliplr(self.map)
        print self.yaml_image_name

    def createYaml(self):
        data = {
            'image': self.yaml_image_name,
            'resolution': float(self.resolution),
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'origin': [0,0.0,0.0]
        }
        with open(self.yaml_name, 'w') as outfile:
            yaml.dump(data, outfile)
    def createPgm(self):
        print len(self.map)
        print len(self.map[0])

        im = Image.new('L',(len(self.map),len(self.map[0])),255)
        pixels = im.load()  # create the pixel map
        for i in range(im.size[0]):  # for every col:
            for j in range(im.size[1]):  # For every row
                if (self.map[i][j] == 1):
                    pixels[i, j] = 0

        #im=im.rotate(90)
        im.save(self.image_name)



if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    input = raw_input("Please enter map path: ")
    resolution = raw_input("Please enter the desired resolution: ")
    map = Map(input,resolution)
    map.storeMap()
    map.readString()
    map.createYaml()
    map.createPgm()


    #rospy.spin()






