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

import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from std_srvs.srv import Empty

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'

class Move():
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('ur3_move', anonymous=True)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        self.br = tf.TransformBroadcaster()

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)

        self.display_trajectory_publisher = display_trajectory_publisher

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)

    def initial_code(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print ("====== planning frame: ", planning_frame)

        self.wpose = self.robot_arm.get_current_pose()
        print ("====== current pose : ", self.wpose)

        joint_goal = [2.3884003162384033, -1.6826804319964808, 0.7239289283752441, -2.2834256331073206, 4.623193740844727, -0.025881592427388966]
        print("init pose : ", self.robot_arm.get_current_pose().pose.position)
        self.robot_arm.go(joint_goal, wait=True)

    def go_to_move(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print ("====== planning frame: ", planning_frame)

        self.wpose = self.robot_arm.get_current_pose()
        print ("====== current pose : ", self.wpose)

        joint_goal = [2.3886876106262207, -1.6729376951800745, 1.6410179138183594, -2.855138603840963, 4.623205661773682, -0.025869671498433888]
        print("move_pose : ", self.robot_arm.get_current_pose().pose.position)
        self.robot_arm.go(joint_goal, wait=True)

    def go_to_tomato(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print("====== planning frame: ", planning_frame)

        self.wpose = self.robot_arm.get_current_pose()
        print("====== current pose : ", self.wpose)

        joint_goal = [2.238821506500244, -0.8660662809955042, 0.8210840225219727, -2.8785727659808558, 4.622917175292969, -0.025917832051412404]
        print("tomato_pose : ", self.robot_arm.get_current_pose().pose.position)
        self.robot_arm.go(joint_goal, wait=True)

    def plan_cartesian_y(self, y_offset, scale=1.0):
        waypoints = []
        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.y = (scale * self.wpose.position.y) + y_offset
        waypoints.append(copy.deepcopy(self.wpose))

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_cmd.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        group = self.robot_arm
        group.execute(plan, wait=True)

if __name__ == '__main__':
    tm = Move()

    rate = rospy.Rate(10.0)

    time.sleep(1)

    # tm.initial_code()

    time.sleep(0.5)
    
    tm.go_to_move()

    time.sleep(0.5)

    tm.go_to_tomato()