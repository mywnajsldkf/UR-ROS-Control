#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)        

        self.ii=0

        global goal_x
        global goal_y
        global goal_z

        global ori_w
        global ori_x
        global ori_y
        global ori_z

        self.distance = 0

        self.tomato = []
        self.position_list = []
        self.orientation_list = []

        self.tomato_pose = Pose()
        self.goalPoseFromTomato = Pose()
        self.br = tf.TransformBroadcaster()

        self.calculated_tomato = Pose()
        self.calculated_tomato_coor = Pose()

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(10)
        self.robot_arm.set_num_planning_attempts(10)

        self.display_trajectory_publisher = display_trajectory_publisher

        rospy.sleep(2)
        
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)

    def move_code(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print "========== plannig frame: ", planning_frame

        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose : ", self.wpose                        

        marker_joint_goal = [2.6482303142547607, -0.3752959410296839, -2.1118043104754847, -0.4801228682147425, -1.4944542090045374, -4.647655431424276]
        print "INIT POSE: ", self.robot_arm.get_current_pose().pose.position
        self.robot_arm.go(marker_joint_goal, wait=True)

    def pose_subscriber(self):
        rospy.loginfo("waiting for pose topic...")

        rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        rospy.Subscriber('/cluster_decomposer/centroid_pose_array',PoseArray,self.collectJsk)

    def go_to_goal_point(self, scale=1.0):
        planning_frame = self.robot_arm.get_planning_frame()

        print(">> robot arm planning frame: \n",planning_frame)
        
        self.calculated_tomato.position.x = (scale*goal_x)
        self.calculated_tomato.position.y = (scale*goal_y)
        self.calculated_tomato.position.z = (scale*goal_z)

        self.calculated_tomato.orientation.w = (scale*ori_w)
        self.calculated_tomato.orientation.x = (scale*ori_x)
        self.calculated_tomato.orientation.y = (scale*ori_y)
        self.calculated_tomato.orientation.z = (scale*ori_z)

        print(">> robot_pose goal frame: ",self.calculated_tomato)
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
    def go_to_designated_coordinate(self):        
        desired_goal_pose = [-0.537,0.166, 0.248]

        Cplanning_frame = self.robot_arm.get_planning_frame()
        print(">> current planning frame: \n",Cplanning_frame)

        self.calculated_tomato_coor.position.x = desired_goal_pose[0]
        self.calculated_tomato_coor.position.y = desired_goal_pose[1]
        self.calculated_tomato_coor.position.z = desired_goal_pose[2]
        self.calculated_tomato_coor.orientation = Quaternion(*quaternion_from_euler(desired_goal_pose[0],desired_goal_pose[1],desired_goal_pose[2]))

        print(">> goal frame", self.calculated_tomato_coor)
        self.robot_arm.set_pose_target(self.calculated_tomato_coor)

        tf_display_position = [self.calculated_tomato_coor.position.x, self.calculated_tomato_coor.position.y, self.calculated_tomato_coor.position.z]
        tf_display_orientation = [self.calculated_tomato_coor.orientation.x, self.calculated_tomato_coor.orientation.y, self.calculated_tomato_coor.orientation.z, self.calculated_tomato_coor.orientation.w]

        jj = 0
        while jj < 5:
            jj += 1
            self.br.sendTransform(
                tf_display_position,
                tf_display_orientation,
                rospy.Time.now(),
                "goal_wpose",
                "world")

        tomato_waypoints = []
        tomato_waypoints.append(copy.deepcopy(self.calculated_tomato_coor))
        (plan, fraction) = self.robot_arm.compute_cartesian_path(tomato_waypoints,0.01,0.0)
        self.display_trajectory(plan)

        print("=========== Press `Enter` to if plan is correct!!...")
        raw_input()
        self.robot_arm.go(True)
    '''

    def plan_cartesian_path(self,scale=1.0):
        waypoints = []

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.x -= scale*0.1
        self.wpose.position.y += scale*0.1 
        waypoints.append(copy.deepcopy(self.wpose))
	
	'''
        self.wpose.position.x -= scale*0.2
        waypoints.append(copy.deepcopy(self.wpose))
    	'''

        '''
        self.wpose.position.x -= scale*0.1
        waypoints.append(copy.deepcopy(self.wpose))
        ''' 

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def execute_plan(self,plan):
        group = self.robot_arm
        group.execute(plan, wait=True)

    def display_trajectory(self, plan):
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_cmd.get_current_state()
        display_trajectory.trajectory.append(plan)
            
        display_trajectory_publisher.publish(display_trajectory)
                         
    def collectJsk(self, msg):
        global goal_x
        global goal_y
        global goal_z

        global ori_w
        global ori_x
        global ori_y
        global ori_z

        try:
            (trans, rot) = self.listener.lookupTransform('base_link','yolo_output00',rospy.Time(0))

            goal_x = trans[0]
            goal_y = trans[1]
            goal_z = trans[2]

            ori_w = rot[0]
            ori_x = rot[1]
            ori_y = rot[2]
            ori_z = rot[3]

            print("==== trans[x,y,z]: ",trans)
            print("==== rot[x,y,z,w]: ",rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('there is no tf')

if __name__=='__main__':
    tm = TestMove()
    rate = rospy.Rate(10.0)
    # tm.move_code()
    time.sleep(1)

    '''	
    print("start cartesian_path")
    cartesian_plan, fraction = tm.plan_cartesian_path()
    tm.display_trajectory(cartesian_plan)
    time.sleep(0.5)
    tm.execute_plan(cartesian_plan)
    '''


    '''
    print("!! go to desired coordinate !!")
    tm.go_to_designated_coordinate()
    time.sleep(0.5)
    '''

    print(">> go to desired coordinate")
    tm.pose_subscriber()
    time.sleep(0.5)
	
    '''
    tm.go_to_goal_point()

    print("go to init pose, press the enter")
    raw_input()
    tm.move_code()
    '''
    rospy.spin()
    roscpp_shutdown()
