#!/usr/bin/python
"""RRT PATH PLANNING"""

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
from geometry_msgs.msg import PoseWithCovarianceStamped
from trabajo_planificacion.srv import *

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import atan2, degrees
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal

from scipy.interpolate import interp1d
import random
import math

path = []
rrt_publisher = rospy.Publisher('visualization_rrt', Marker, queue_size=10)
#rrt_path_publisher = rospy.Publisher('rrt_plan', Path, queue_size=10)


def timing(f):
    def wrap(*args):
        time1 = time.time()
        ret = f(*args)
        time2 = time.time()
        print '%s function took %0.3f ms' % (f.func_name, (time2 - time1) * 1000.0)
        return ret

    return wrap


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT():
    def __init__(self):
        self.map = []
        self.res = 1
        # Inicializo los nodos
        self.start = Node(0.0, 0.0)
        self.final = Node(0.0, 0.0)
        self.max_iterations = 50000
        self.error = 0.5

        self.marker_rrt_edges = Marker(id=0, header=Header(frame_id='map', stamp=rospy.Time.now()))
        self.marker_rrt_edges.type = Marker.LINE_LIST
        self.marker_rrt_edges.action = Marker.ADD

        self.marker_rrt_edges.scale.x = 0.04

        # marker color
        self.line_color = ColorRGBA()
        self.line_color.a = 0.5
        self.line_color.r = 1.0

    @timing
    def storeMap(self):
        self.map = []
        with open(self.csvfile, 'rb') as csvfile:
            self.map = np.loadtxt(self.csvfile, dtype=int, delimiter=",")

    @timing
    def readMap(self, map):
        for row in map:
            print row

    def compute_rrt(self):
        success = False
        # input_x_node = float(raw_input("x_node: "))
        # input_y_node = float(raw_input("y_node: "))
        # self.start = Node(input_x_node, input_y_node)
        #
        # input_x_node = float(raw_input("x_node: "))
        # input_y_node = float(raw_input("y_node: "))
        # self.final = Node(input_x_node, input_y_node)
        self.nodes = [self.start]
        if self.map[int(self.start.x / self.res)][int(self.start.y / self.res)] == 1:
            print "Start is in an obstacle"
            return False
        if self.map[int(self.final.x / self.res)][int(self.final.y / self.res)] == 1:
            print "Goal is in an obstacle"
            return False
        for iter in range(self.max_iterations):
            if not success:
                # Generate a random config
                x = random.uniform(0, len(self.map) * self.res)
                y = random.uniform(0, len(self.map[0]) * self.res)
                print x, y
                randomNode = Node(x, y)
                # Get nearest node
                nearest_ind = self.get_nearest_node(randomNode)
                nearestNode = self.nodes[nearest_ind]
                # nearestNode = Node(input_x_node,input_y_node)
                if self.check_collision(randomNode, nearestNode):
                    continue
                randomNode.parent = nearest_ind
                self.nodes.append(randomNode)
                self.visualize_rrt(nearestNode, randomNode)
                if abs(randomNode.x - self.final.x) < self.error and abs(randomNode.y - self.final.y) < self.error:
                    success = True
                    print "success"
                    return True
        print("More iterations needed")
        return False

    def check_collision(self, randomNode, nearestNode):
        collision = False
        if (randomNode.x != nearestNode.x):
            n = (randomNode.y - nearestNode.y)
            d = (randomNode.x - nearestNode.x)
            m = n / d
            theta = math.atan(m)

            if theta < 0:
                if d < 0:
                    theta = theta + math.pi
                else:
                    theta = theta + 2 * math.pi
            else:
                if n < 0 and d < 0:
                    theta = theta + math.pi

            sin_theta = math.sin(theta)
            cos_theta = math.cos(theta)
            for step in range(int(math.sqrt(n * n + d * d) / self.res)):
                if self.map[int(randomNode.x / self.res + step * cos_theta / self.res)][
                    int(randomNode.y / self.res + step * sin_theta / self.res)] == 1:
                    return True
        else:
            for step in range(int((randomNode.y - nearestNode.y) / self.res)):
                if self.map[int(randomNode.x / self.res)][randomNode.y + step] == 1:
                    return True

    def get_nearest_node(self, random_node):
        dlist = [(node.x - random_node.x) ** 2 + (node.y - random_node.y)
                 ** 2 for node in self.nodes]
        min_in = dlist.index(min(dlist))
        return min_in

    def visualize_rrt(self, nearestNode, randomNode):
        p1 = Point()
        p1.x = nearestNode.x
        p1.y = nearestNode.y
        p1.z = 0.0

        self.marker_rrt_edges.points.append(p1)
        # print marker_voro_edges
        self.marker_rrt_edges.colors.append(self.line_color)

        p2 = Point()
        p2.x = randomNode.x
        p2.y = randomNode.y
        p2.z = 0.0
        self.marker_rrt_edges.points.append(p2)
        # print marker_voro_edges
        self.marker_rrt_edges.colors.append(self.line_color)

        rrt_publisher.publish(self.marker_rrt_edges)

    def compute_path(self,req):
        self.start.x = req.start.pose.position.x
        self.start.y = req.start.pose.position.y
        self.final.x = req.final.pose.position.x
        self.final.y = req.final.pose.position.y

        self.marker_rrt_edges = Marker(id=0, header=Header(frame_id='map', stamp=rospy.Time.now()))
        self.marker_rrt_edges.type = Marker.LINE_LIST
        self.marker_rrt_edges.action = Marker.ADD

        self.marker_rrt_edges.scale.x = 0.04

        # marker color
        self.line_color = ColorRGBA()
        self.line_color.a = 0.5
        self.line_color.r = 1.0

        res = path_calcResponse()
        res.result = 0
        if self.compute_rrt()==False:
            return res
        path = [[self.final.x, self.final.y]]
        lastIndex = len(self.nodes) - 1
        while self.nodes[lastIndex].parent is not None:
            node = self.nodes[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        path.reverse()

        new_path = self.path_smoothing(path,1000)
        poseArray = PoseArray()
        #path_a = Path()
        for p in new_path:
             pose = Pose()
        #     pose_stamp=PoseStamped()
        #     pose_stamp.pose.position.x = p[0]
        #     pose_stamp.pose.position.y = p[1]
             pose.position.x = p[0]
             pose.position.y = p[1]
        #     path_a.poses.append(pose_stamp)
             poseArray.poses.append(pose)
        #rrt_path_publisher.publish(path_a)
        res.path = poseArray
        print path

        res.result = 1
        return res
    def GetPathLength(self,path):
        le = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d

        return le

    def callback_map(self, data):
        self.res = data.info.resolution
        self.map = np.zeros((data.info.width, data.info.height), dtype=int)
        self.readMap(self.map)
        for i in range(0, data.info.width):
            for j in range(0, data.info.height):
                # print data.data[j * data.info.width + i]
                if data.data[j * data.info.width + i] > 0:
                    self.map[i][j] = 1
        self.readMap(self.map)
        #self.compute_rrt()
    # def callback_start(self,data):
    #     self.start.x = data.pose.pose.position.x
    #     self.start.y = data.pose.pose.position.y
    #
    # def callback_goal(self,data):
    #     self.final.x = data.goal.target_pose.pose.position.x
    #     self.final.y = data.goal.target_pose.pose.position.y
    #     self.compute_path()

    def GetTargetPoint(self,path, targetL):
        le = 0
        ti = 0
        lastPairLen = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d
            if le >= targetL:
                ti = i - 1
                lastPairLen = d
                break

        partRatio = (le - targetL) / lastPairLen
        #  print(partRatio)
        #  print((ti,len(path),path[ti],path[ti+1]))

        x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
        y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
        #  print((x,y))

        return [x, y, ti]
    def path_smoothing(self,path, maxIter):
        #  print("PathSmoothing")

        le = self.GetPathLength(path)

        for i in range(maxIter):
            # Sample two points
            pickPoints = [random.uniform(0, le), random.uniform(0, le)]
            pickPoints.sort()
            #  print(pickPoints)
            first = self.GetTargetPoint(path, pickPoints[0])
            #  print(first)
            second = self.GetTargetPoint(path, pickPoints[1])
            #  print(second)

            if first[2] <= 0 or second[2] <= 0:
                continue

            if (second[2] + 1) > len(path):
                continue

            if second[2] == first[2]:
                continue

            # collision check
            node1=Node(first[0],first[1])
            node2=Node(second[0],second[1])

            if self.check_collision(node1, node2):
                continue

            # Create New path
            newPath = []
            newPath.extend(path[:first[2] + 1])
            newPath.append([first[0], first[1]])
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            le = self.GetPathLength(path)
        return path

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    # input = raw_input("Please enter map path: ")
    # map = Map(input)
    rrt = RRT()
    rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, rrt.callback_map)
    # rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, rrt.callback_goal)
    # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, rrt.callback_start)

    # map.storeMap()
    # map.readMap(map.map)
    # map.createVoronoi()
    # map.visualize_voronoi()
    rospy.Service('path_calc', path_calc, rrt.compute_path)
    while not rospy.is_shutdown():
        rospy.spin()
        rospy.sleep(1)
