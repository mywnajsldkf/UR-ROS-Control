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

#############################################
# Global Variables                          #
#############################################
bbox    = BoundingBox()
pre_bbox= BoundingBox()

pc      = PointCloud2()
center  = Vector3()
point   = Vector3()
points  = []
depth   = 0.0

def bbox_callback(msg):
    num_box = len(msg.bounding_boxes)
    if num_box>0:
        b         = msg.bounding_boxes[0]
        bbox.xmin = b.xmin
        bbox.xmax = b.xmax
        bbox.ymin = b.ymin
        bbox.ymax = b.ymax

def point_callback(msg):
    global points
    global bbox
    global pre_bbox
    pc.header = msg.header
    pc.height = msg.height
    pc.width  = msg.width
    pc.fields = msg.fields      # channels and their layout
    pc.is_bigendian = msg.is_bigendian
    pc.point_step   = msg.point_step
    pc.row_step     = msg.row_step
    pc.data         = msg.data  #  Actual point data, size is (row_step*height)
    resolution = (pc.height, pc.width)


    if bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin and \
        bbox.xmin==pre_bbox.xmin:
        pass
    else:
        points = [  ]
        for p in pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z")):
            points.append(p[2])
        if len(points) == pc.width*pc.height:
            z_points = np.array(points, dtype=np.float32)
            z = z_points.reshape(resolution)

            if not (bbox.xmin==0 and bbox.xmax==0):
                # print('Box: {}, {}'.format(bbox.xmin, bbox.xmax))
		
		'''
		# jeongin added
                x_center=(bbox.xmax+bbox.xmin)/2
                y_center=(bbox.ymax+bbox.ymin)/2
                print('Box_Center: x {}, y {}'.format(x_center,y_center))

                z_box = z[bbox.xmin:bbox.xmax, bbox.ymin:bbox.ymax]
                z_value = z_box[~np.isnan(z_box)]
                distance = min(z_value)
                print('Distance: {}'.format(distance))
		'''

        pre_bbox.xmin = bbox.xmin
        pre_bbox.xmax = bbox.xmax
        pre_bbox.ymin = bbox.ymin
        pre_bbox.ymax = bbox.ymax


class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('ur3_move', anonymous=True)

        rospy.Subscriber('camera/depth_registered/points', PointCloud2, point_callback)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, bbox_callback)

        freq = 30
        rate = rospy.Rate(freq)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        self.ii = 0

        # self.listener = tf.TransformListener()
        self.goal_x = 0
        self.goal_y = 0 
        self.goal_z = 0

        self.goal_ori_x = 0
        self.goal_ori_y = 0
        self.goal_ori_z = 0
        self.goal_ori_w = 0

        self.marker = []
        self.position_list = []
        self.orientation_list = []

        self.m_idd = 0
        self.m_pose_x = []
        self.m_pose_y = []
        self.m_pose_z = []
        self.m_ori_w = []
        self.m_ori_x = []
        self.m_ori_y = []
        self.m_ori_z = []
        
        self.br = tf.TransformBroadcaster()

        self.target_ar_id = 9

        self.calculed_tomato = Pose()

        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

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

        self.clear_octomap()

    def move_code(self):
        planning_frame = self.robot_arm.get_planning_frame()
        print ("====== planning frame: ", planning_frame)

        self.wpose = self.robot_arm.get_current_pose()
        print ("====== current pose : ", self.wpose)

        joint_goal = [-0.535054565144069, -2.009213503260451, 1.8350906250920112, -0.7794355413099039, -0.7980899690645948, 0.7782740454087982]
        print ("INIT POSE: ", self.robot_arm.get_current_pose().pose.position)
        self.robot_arm.go(joint_goal, wait = True)

    def go_to_move(self, scale = 1.0):
        '''
        manipulator is moving to the target ar_marker.
        start ar_marker id is 9, 
        goal ar_marker id is 10
        '''

        planning_frame = self.robot_arm.get_planning_frame()
        tomato_offset = [0,-0.35,-0.1] # x y z
        robot_base_offset = 0.873
        base_wrist2_offset = 0.1

        # start ar_marker
        if self.target_ar_id == 9:
            print (">> robot arm planning frame: \n", planning_frame)
            print (">> move mode id: ", self.target_ar_id)

            self.calculed_tomato.position.x = (scale*self.goal_x)   # base_link to wrist2 x-offset
            self.calculed_tomato.position.y = (scale*self.goal_y) + tomato_offset[1]
            self.calculed_tomato.position.z = (scale*self.goal_z) + robot_base_offset
            self.calculed_tomato.orientation = Quaternion(*quaternion_from_euler(3.14,0,1.57))

            print ("======== robot_pose goal frame: ", self.calculed_tomato)
            self.robot_arm.set_pose_target(self.calculed_tomato)  
        
        # goal ar_marker
        elif self.target_ar_id == 10:
            print (">> robot arm plannig frame: \n", planning_frame)
            print (">> move mode id: ", self.target_ar_id)

            self.calculed_tomato.position.x = (scale*self.goal_x) + tomato_offset[1]   # base_link to wrist2 x-offset
            self.calculed_tomato.position.y = (scale*self.goal_y)
            self.calculed_tomato.position.z = (scale*self.goal_z) + robot_base_offset
            self.calculed_tomato.orientation = Quaternion(*quaternion_from_euler(3.14,0,1.57))

            print ("======== robot_pose goal frame: ", self.calculed_tomato)
            self.robot_arm.set_pose_target(self.calculed_tomato)

        tf_display_position = [self.calculed_tomato.position.x, self.calculed_tomato.position.y, self.calculed_tomato.position.z]
        tf_display_orientation = [self.calculed_tomato.orientation.x, self.calculed_tomato.orientation.y, self.calculed_tomato.orientation.z, self.calculed_tomato.orientation.w]

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

        ### show how to move on the Rviz
        tomato_waypoints = []
        tomato_waypoints.append(copy.deepcopy(self.calculed_tomato))
        (tomato_plan, tomato_fraction) = self.robot_arm.compute_cartesian_path(tomato_waypoints, 0.01, 0.0)       
        self.display_trajectory(tomato_plan)

        print ("============ Press `Enter` to if plan is correct!! ...")
        raw_input()
        self.robot_arm.go(True)    

    def go_to_desired_coordinate(self, pose_x, pose_y, pose_z, roll, pitch, yaw):
        '''
        Manipulator is moving to the desired coordinate
        Now move to the ar_10 marker
        '''
        calculated_ar_id_10 = Pose()
        desired_goal_pose = [pose_x, pose_y, pose_z]
        desired_goal_euler = [roll, pitch, yaw]

        Cplanning_frame = self.robot_arm.get_planning_frame()
        print (">> current planning frame: \n", Cplanning_frame)

        calculated_ar_id_10.position.x = desired_goal_pose[0] + 0.1
        calculated_ar_id_10.position.y = desired_goal_pose[1]
        calculated_ar_id_10.position.z = desired_goal_pose[2]
        calculated_ar_id_10.orientation = Quaternion(*quaternion_from_euler(desired_goal_euler[0], desired_goal_euler[1],desired_goal_euler[2]))

        print (">>> ar id 10 goal frame: ", desired_goal_pose)
        self.robot_arm.set_pose_target(calculated_ar_id_10)

        tf_display_position = [calculated_ar_id_10.position.x, calculated_ar_id_10.position.y, calculated_ar_id_10.position.z]
        tf_display_orientation = [calculated_ar_id_10.orientation.x, calculated_ar_id_10.orientation.y, calculated_ar_id_10.orientation.z, calculated_ar_id_10.orientation.w]

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

            ## show how to move on the Rviz
            ar_id_10_waypoints = []
            ar_id_10_waypoints.append(copy.deepcopy(calculated_ar_id_10))
            (ar_id_10_plan, ar_id_10_fraction) = self.robot_arm.compute_cartesian_path(ar_id_10_waypoints,0.01, 0.0)
            self.display_trajectory(ar_id_10_plan)
            ##

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

        rospy.Subscriber('camera/depth_registered/points', PointCloud2, point_callback)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, bbox_callback)

if __name__ == '__main__':
    '''
    shoulder_pan_joint = 0
    shoulder_lift_joint = 1
    elbow_joint = 2
    wrist_1_joint = 3
    wrist_2_joint = 4
    wrist_3_joint = 5
    '''

    tm = TestMove()
    
    rate = rospy.Rate(10.0)
    tm.move_code()      # go to initial pose
    time.sleep(1)

    tm.go_to_move()

    linear_path = [-0.0001, -0.0625, -0.08]
    print (">> Linear path planning 01 - pick the object")
    cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0], linear_path[1], linear_path[2]) # set value on every parameters
    tm.display_trajectory(cartesian_plan)
    time.sleep(0.5)
    tm.execute_plan(cartesian_plan) 
    print(">> Press the Enter key")
    raw_input()

    linear_path = [+0.2, -0.2, +0.2]
    print ">> Linear path planning 02-z"
    cartesian_plan, fraction = tm.plan_cartesian_z(linear_path[2]) # set variable to all parameters
    tm.display_trajectory(cartesian_plan)
    time.sleep(2)
    tm.execute_plan(cartesian_plan)

    ar_id10_goal_pose = [0.171, -0.113, 1.039]
    ar_id10_goal_euler = [3.14, 0.17, 0]

    tm.go_to_desired_coordinate(ar_id10_goal_pose[0],ar_id10_goal_pose[1],ar_id10_goal_pose[2],ar_id10_goal_euler[0],ar_id10_goal_euler[1],ar_id10_goal_euler[2])

    tm.target_ar_id = 10
    tm.pose_subscriber()
    time.sleep(0.5)

    tm.go_to_move()

    linear_path = [-0.0625, 0.0001, -0.08]
    print ">> Linear path planning 03-drop the object"
    cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0], linear_path[1], linear_path[2]) 
    tm.display_trajectory(cartesian_plan)
    time.sleep(0.5)
    tm.execute_plan(cartesian_plan)
    print ">> Press the Enter key"
    raw_input()

    print ("Go to Init pose, Press the Enter")
    raw_input()
    tm.move_code()
