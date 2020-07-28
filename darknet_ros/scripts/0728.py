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

        # self.object_name_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.collect)

        self.tomatoboundingBox = BoundingBox()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)        

        global goal_x
        global goal_y
        global goal_z

        global goal_roll
        global goal_pitch
        global goal_yaw

        self.distance = 0

        self.trans = [0.0, 0.0, 0.0]

#        self.marker = []
        self.tomato = []
        self.position_list = []
        self.orientation_list = []

        self.tomato_pose = Pose()
        self.goalPoseFromTomato = Pose()
        self.br = tf.TransformBroadcaster()

        self.calculated_tomato = Pose()

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

    def move_code(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print ("====== planning frame: ", planning_frame)

        self.wpose = self.robot_arm.get_current_pose()
        print ("====== current pose : ", self.wpose)

        joint_goal = [-0.535054565144069, -2.009213503260451, 1.8350906250920112, -0.7794355413099039, -0.7980899690645948, 0.7782740454087982]
        print ("INIT POSE: ", self.robot_arm.get_current_pose().pose.position)
        self.robot_arm.go(joint_goal, wait = True)


    def go_to_desired_coordinate(self):
        self.calculated_tomato.position.x = goal_x
        self.calculated_tomato.position.y = goal_y 
        self.calculated_tomato.position.z = goal_z

        self.calculated_tomato.orientation.x = goal_roll
        self.calculated_tomato.orientation.y = goal_pitch
        self.calculated_tomato.orientation.z = goal_yaw

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
            rate.sleep()

        ## ## ## show how to move on the Rviz
        tomato_id_goal_waypoints = []
        tomato_id_goal_waypoints.append(copy.deepcopy(calculated_tomato))
        (tomato_id_goal_plan, tomato_id_goal_fraction) = self.robot_arm.compute_cartesian_path(tomato_id_goal_waypoints, 0.01, 0.0)
        self.display_trajectory(tomato_id_goal_plan)

        print ("============ Press `Enter` to if plan is correct!! ...")
        raw_input()
        self.robot_arm.go(True)

    def display_trajectory(self, plan):
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_cmd.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def plan_cartesian_path(self, x_offset, y_offset, z_offset, scale = 1.0):
        waypoints = []
        ii = 1

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.x = (scale * self.wpose.position.x) + x_offset      # - 0.10

        waypoints.append(copy.deepcopy(self.wpose)) 
        ii += 1

        self.wpose.position.y = (scale * self.wpose.position.y) + y_offset      # + 0.05
        waypoints.append(copy.deepcopy(self.wpose))
        ii += 1

        self.wpose.position.z = (scale * self.wpose.position.z) + z_offset
        waypoints.append(copy.deepcopy(self.wpose))
        ii += 1

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def plan_cartesian_x(self, x_offset, scale=1.0):
        waypoints = []

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.x = (scale * self.wpose.position.x) + x_offset      # -0.10
        waypoints.append(copy.deepcopy(self.wpose))

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def plan_cartesian_y(self, y_offset, scale=1.0):
        waypoints = []

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.y = (scale * self.wpose.position.y) + y_offset      # -0.10
        waypoints.append(copy.deepcopy(self.wpose))

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def plan_cartesian_z(self, z_offset, scale=1.0):
        waypoints = []

        self.wpose = self.robot_arm.get_current_pose().pose
        self.wpose.position.z = (scale * self.wpose.position.z) + z_offset      # -0.10
        waypoints.append(copy.deepcopy(self.wpose))

        (plan, fraction) = self.robot_arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def execute_plan(self, plan):
        group = self.robot_arm
        group.execute(plan, wait=True)

    def pose_subscriber(self):
        rospy.loginfo("waiting for pose topic...")

        rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        rospy.Subscriber('/cluster_decomposer/centroid_pose_array',PoseArray,self.collectJsk)
        # rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.collect)


    def collectJsk(self, msg):
        global goal_x
        global goal_y
        global goal_z

        global goal_roll
        global goal_pitch
        global goal_yaw

        try:
            (trans, rot) = self.listener.lookupTransform('base_link','yolo_output00', rospy.Time(0))

            goal_x = round(trans[0],2)
            goal_y = round(trans[1],2)
            goal_z = round(trans[2],2)

            print("====== trans [x, y, z]: ", trans)
            print("====== rotat [x, y, z, w]: ", rot)

            orientation = euler_from_quaternion(rot)

            goal_roll = orientation[0]
            goal_pitch = orientation[1]
            goal_yaw = orientation[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('there is no tf')

if __name__ == '__main__':
    tm = TestMove()

    rate = rospy.Rate(10.0)

    # go to initial pose
    # tm.move_code()
    # time.sleep(1)

    tm.pose_subscriber()
    time.sleep(0.5)

    tm.go_to_desired_coordinate()

    linear_path = [+0.2, -0.2, +0.2]
    print(">> start path planning")
    cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0],linear_path[1],linear_path[2])
    tm.display_trajectory(cartesian_plan)
    time.sleep(0.5)
    tm.execute_plan(cartesian_plan)

    print (">> Press the Enter key <<")
    raw_input()
    tm.move_code()  # go to initial pose


'''
[ERROR] [1595941425.326841225, 160.607000000]: Ignoring transform for child_frame_id "goal_wpose" from authority "/ur3_move_16491_1595941414908" because of an invalid quaternion in the transform (1.568695 0.017949 3.140640 0.000000)
[ERROR] [1595941425.327733077, 160.607000000]: Ignoring transform for child_frame_id "goal_wpose" from authority "unknown_publisher" because of an invalid quaternion in the transform (1.568695 0.017949 3.140640 0.000000)


Traceback (most recent call last):
  File "/home/tx2/catkin_ws/src/darknet_ros/darknet_ros/scripts/0728.py", line 254, in <module>
    tm.go_to_desired_coordinate()
  File "/home/tx2/catkin_ws/src/darknet_ros/darknet_ros/scripts/0728.py", line 131, in go_to_desired_coordinate
    tomato_id_goal_waypoints.append(copy.deepcopy(calculated_tomato))
NameError: global name 'calculated_tomato' is not defined

'''
