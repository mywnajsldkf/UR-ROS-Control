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
        self.listener = tf.TransformListener()

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

    def move_code(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print "========== plannig frame: ", planning_frame

        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(1) : ", self.wpose               

        marker_joint_goal = [4.721744537353516, -0.7451499144183558, -1.6199515501605433, -1.2175200621234339, 1.6366002559661865, 3.1263363361358643]  
        print "INIT POSE: ", self.robot_arm.get_current_pose().pose.position
        self.robot_arm.go(marker_joint_goal, wait=True)
    
        self.wpose = self.robot_arm.get_current_pose()
        print"====== current pose(2) : ", self.wpose 

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
    tm = TestMove()
    rate = rospy.Rate(10)
    time.sleep(1)
    tm.move_code()
    time.sleep(1)

    rospy.spin()
    roscpp_shutdown()
