#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import struct
import argparse
import time
import sys
import tf
import copy, math
import numpy as np
import matplotlib.pyplot    as plt
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg   import BoundingBox, BoundingBoxes
from geometry_msgs.msg      import Vector3

import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from math import pi, pow, atan2, sqrt

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import * 
from enum import Enum
import random

from std_srvs.srv import Empty

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'

class TestMove():
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('ur3_move', anonymous=True)
      
        self.detected = {}
        self.detected_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        self.object_pose_sub = rospy.Subscriber('/cluster_decomposer/centroid_pose_array',PoseArray,self.collectJsk)
        self.listener = tf.TransformListener()

        self.clear_octomap = rospy.ServiceProxy("/clear_octomap",Empty)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)        

        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0

        self.goal_ori_x = 0
        self.goal_ori_y = 0
        self.goal_ori_z = 0
        self.goal_ori_w = 0

        self.position_list = []
        self.orientation_list = []

        self.goalPoseFromTomato = Pose()
        self.br = tf.TransformBroadcaster()

        self.calculated_tomato = Pose()

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.robot_arm.set_goal_orientation_tolerance(0.5)
        self.robot_arm.set_planning_time(10)
        self.robot_arm.set_num_planning_attempts(10)

        self.display_trajectory_publisher = display_trajectory_publisher

        rospy.sleep(2)
        
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)

        self.clear_octomap()

    def move_code(self):
        '''
        planning_frame = self.robot_arm.get_planning_frame()
        print "========== plannig frame: ", planning_frame

        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(1) : ", self.wpose               
        '''
        marker_joint_goal = [4.721744537353516, -0.7451499144183558, -1.6199515501605433, -1.2175200621234339, 1.6366002559661865, 3.1263363361358643]  
        print "INIT POSE: ", self.robot_arm.get_current_pose().pose.position
        self.robot_arm.go(marker_joint_goal, wait=True)
        '''
        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(2) : ", self.wpose 
        '''

    def move_code2(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print "========== plannig frame: ", planning_frame

        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(1) : ", self.wpose
        
        marker_joint_goal = [4.768651485443115, -1.1165898481952112, -2.1672890822040003, 5.898628234863281, 1.7003079652786255, 3.2297513484954834]
        print "INIT POSE: ", self.robot_arm.get_current_pose().pose.position
        self.robot_arm.go(marker_joint_goal, wait=True)

        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(2) : ", self.wpose 

    def plan_cartesian_path(self,scale=1.0):
        waypoints = []

        '''
        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(1) : ", self.wpose
        '''

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.x += scale*0.009
        self.wpose.position.y += scale*0.07
        self.wpose.position.z -= scale*0.06
        waypoints.append(copy.deepcopy(self.wpose))

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        '''
        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(2) : ", self.wpose 
        '''

        return plan, fraction

    def pose_subscriber(self):
        rospy.loginfo("waiting for pose topic...")

        rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        
        detection_start = time.time()
        rospy.Subscriber('/cluster_decomposer/centroid_pose_array',PoseArray,self.collectJsk)
        print("detection time",format(time.time()-detection_start))

    detection_start = time.time()
    def collectJsk(self, msg):
        # detection_start = time.time()

        try:
            detection_start = time.time()
            (trans, rot) = self.listener.lookupTransform('base_link','yolo_output00',rospy.Time(0))

            self.goal_x = trans[0]
            self.goal_y = trans[1]
            self.goal_z = trans[2]

            self.goal_ori_x = rot[0]
            self.goal_ori_y = rot[1]
            self.goal_ori_z = rot[2]
            self.goal_ori_w = rot[3]
 
            self.position_list = [self.goal_x, self.goal_y, self.goal_z]
            self.orientation_list = [self.goal_ori_x, self.goal_ori_y, self.goal_ori_z, self.goal_ori_w]
            (self.goal_roll, self.goal_pitch, self.goal_yaw) = euler_from_quaternion(self.orientation_list)

        except:
            return
    print("detection time",format(time.time()-detection_start))
    
        # print("detection time",format(time.time()-detection_start))


    def go_to_move(self, scale=1.0):
        '''
        planning_frame = self.robot_arm.get_planning_frame()

	    self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(1) : ", self.wpose
        '''

        self.calculated_tomato.position.x = (scale*self.goal_x)
        self.calculated_tomato.position.y = (scale*self.goal_y)
        self.calculated_tomato.position.z = (scale*self.goal_z)
        self.calculated_tomato.orientation = Quaternion(*quaternion_from_euler(3.14, 0, 1.57))

        print("============ tomato_pose goal frame: ", self.calculated_tomato)
        self.robot_arm.set_pose_target(self.calculated_tomato)

        tf_display_position = [self.calculated_tomato.position.x, self.calculated_tomato.position.y, self.calculated_tomato.position.z]
        tf_display_orientation = [self.calculated_tomato.orientation.x, self.calculated_tomato.orientation.y, self.calculated_tomato.orientation.z, self.calculated_tomato.orientation.w]

        ii = 0
        while ii < 5:
            ii += 1
            self.br.sendTransform(
                tf_display_position,
                tf_display_orientation,
                rospy.Time.now(),
                "goal_wpose",
                "world")

        tomato_waypoints = []
        tomato_waypoints.append(copy.deepcopy(self.calculated_tomato))
        (tomato_plan, tomato_fraction) = self.robot_arm.compute_cartesian_path(tomato_waypoints, 0.01, 0.0)
        self.display_trajectory(tomato_plan)

        print("======= Press `Enter` to if plan in correct!======")
        raw_input()
        self.robot_arm.go(True)

        '''
        self.wpose = self.robot_arm.get_current_pose()
            print"====== current pose(2) : ", self.wpose
        '''

    def execute_plan(self,plan):
        group = self.robot_arm
        group.execute(plan, wait=True)

    def display_trajectory(self, plan):
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_cmd.get_current_state()
        display_trajectory.trajectory.append(plan)
            
        display_trajectory_publisher.publish(display_trajectory)
                         
if __name__=='__main__':
    i = 1

    while not rospy.is_shutdown():
        tm = TestMove()

        print(">> ",i,"episode start!")
        i = i+1
        
        time.sleep(5)

        rate = rospy.Rate(10)
        time.sleep(5)
        
        init_start = time.time()
        tm.move_code()
        print("go to init time : ", format(time.time() - init_start))
        time.sleep(3)
        
        cartesian_start = time.time()
        print(">> start cartesian_path")
        cartesian_plan, fraction = tm.plan_cartesian_path()
        tm.display_trajectory(cartesian_plan)
        tm.execute_plan(cartesian_plan)
        print("cartesian time : ", format(time.time() - cartesian_start))

        '''
        
        #print(">> go to specific joint states")
        #tm.move_code2()
        #time.sleep(3)
        '''

        subscribe = time.time()
        tm.pose_subscriber()
        print("subscriber time : ", format(time.time() - subscribe))

        print(">> go to desired coordinate")

        time.sleep(1)
        move_start = time.time()
        tm.go_to_move()
        print("go to goal time : ", format(time.time() - move_start))

        time.sleep(5)

        print("go to init pose, press the enter")
        raw_input()
        # point_start = time.time()
        tm.move_code()
        # print("go to init time : ", format(time.time()-point_start))

    rospy.spin()
    roscpp_shutdown()
