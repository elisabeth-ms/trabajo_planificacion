#!/usr/bin/env python

import rospy
import time
from math import pow, atan2,sqrt
import tf
from std_msgs.msg import String
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class TimePerformance(object):
    def __init__(self):
        self.pathsub = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan',Path,self.pathcallback)
        self.statsub = rospy.Subscriber('/move_base/status',GoalStatusArray,self.statuscallback)
        self.goalsub = rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,self.goalcallback)
        self.posesub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.posecallback)
        # time
        self.start = 0
        self.end = 0
        # pose
        self.current_position = [0, 0]
        self.current_yaw = 0
        self.deltaposition = 0
        #status
        self.status = 0
        self.ini = 0

    def __enter__(self):
        rospy.loginfo('Initialicing Time Performance measurement')
        #self.wait()
        return self

    def wait(self):
        rate = rospy.Rate(10) # 10hz
        while self.status != 3 and self.ini == 1:
            rate.sleep()
        self.end = time.time()
        self.ini = 0
        rospy.loginfo(" tiempo = %d", self.end-self.start)
        rospy.loginfo(" distancia = %d", self.deltaposition)
        self.deltaposition = 0


    def statuscallback(self,data):
        self.status = data.status_list[0].status
        rospy.logdebug(" status = %d", data.status_list[0].status)

    def pathcallback(self,data):
        self.pathsub.unregister()
        #self.wait()

    def goalcallback(self,data):
        rospy.logdebug('Received goal')
        self.ini = 1
        self.status = 1
        self.goal_position = [data.goal.target_pose.pose.position.x,
                                data.goal.target_pose.pose.position.y]
        quaternion = (data.goal.target_pose.pose.orientation.x,
                      data.goal.target_pose.pose.orientation.y,
                      data.goal.target_pose.pose.orientation.z,
                      data.goal.target_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.goal_yaw = euler[2]
        self.start = time.time()
        rospy.loginfo ('starting time measure')
        self.wait()

    def posecallback(self,data):
        rospy.logdebug('Received pose')
        position = [data.pose.pose.position.x,
                    data.pose.pose.position.y]

        quaternion = (data.pose.pose.orientation.x,
                      data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z,
                      data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]

        if self.status == 1:
            self.deltaposition += self.euclidean_dist(self.current_position,position)
            print self.deltaposition
        self.current_position = position


    def euclidean_dist(self,current,goal):
        eu = sqrt(pow((goal[0]-current[0]),2)+pow((goal[1]-current[1]),2))
        return eu


    def __exit__(self, exc_type, exc_val, exc_tb):
        rospy.loginfo('Closing time performance measurement')
        self.end = time.time()
        print self.start-self.end




if __name__ == "__main__":
    rospy.init_node('measure_performance', log_level=rospy.DEBUG)

    with TimePerformance() as tp:
        pass
        #tp.wait()

    rospy.spin()
