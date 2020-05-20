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

    global goal_x
    global goal_y
    global goal_z

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
                print('Box: {}, {}'.format(bbox.xmin, bbox.xmax))
				
		# jeongin added
                x_center=(bbox.xmax+bbox.xmin)/2
                y_center=(bbox.ymax+bbox.ymin)/2
                #print('Box_Center: x {}, y {}'.format(x_center,y_center))

                z_box = z[bbox.xmin:bbox.xmax, bbox.ymin:bbox.ymax]
                z_value = z_box[~np.isnan(z_box)]
                distance = min(z_value)
                #print('Distance: {}'.format(distance))
                
                goal_x = x_center/1000
                goal_y = y_center/1000
                goal_z = distance

        pre_bbox.xmin = bbox.xmin
        pre_bbox.xmax = bbox.xmax
        pre_bbox.ymin = bbox.ymin
        pre_bbox.ymax = bbox.ymax

class TestMove():
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('ur3_move',anonymous=True)

        rospy.Subscriber('camera/depth_registered/points',PointCloud2, point_callback)
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, bbox_callback)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        global goal_x
        global goal_y
        global goal_z

        self.goal_ori_x = 0
        self.goal_ori_y = 0
        self.goal_ori_z = 0
        self.goal_ori_w = 0

        
        # self.marker
        self.tomato = []
        # goal
        self.position_list = []
        # goal_ori
        self.orientation_list = []

        self.tomato_pose = Pose()
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


    def go_to_move(self, scale = 1.0):
        planning_frame = self.robot_arm.get_planning_frame()

        tomato_offset = [0,-0.35, -0.1]
        robot_base_offset = 0
        base_wrist2_offset = 0.1

        print(">> robot arm planning frame: \n", planning_frame)

        self.calculated_tomato.position.x = (scale*goal_x)
        self.calculated_tomato.position.y = (scale*goal_y)
        self.calculated_tomato.position.z = (scale*goal_z)

        self.calculated_tomato.orientation = Quaternion(*quaternion_from_euler(3.14, 0.1, 1.57))

        print("=== robot_pose goal frame: ", self.calculated_tomato)
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
            rate.sleep()

        tomato_waypoints = []
        tomato_waypoints.append(copy.deepcopy(self.calculated_tomato))
        (tomato_plan, tomato_offset) = self.robot_arm.compute_cartesian_path(tomato_waypoints,0.01, 0.0)
        self.display_trajectory(tomato_plan)

        print("=== Press `Enter` to if plan is correct!! ...")
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
       
    def point_callback(self, msg):
        self.bounding_boxes = msg.bounding_box
        
        num_box = len(msg.bounding_boxes)
        
        self.tomato_pose.position.x = self.tomato[num_box].pose.pose.position.x
        self.tomato_pose.position.y = self.tomato[num_box].pose.pose.position.y
        self.tomato_pose.position.z = self.tomato[num_box].pose.pose.position.z
        self.tomato_pose.orientation.x = self.tomato[num_box].pose.pose.orientation.x
		self.tomato_pose.orientation.y = self.tomato[num_box].pose.pose.orientation.y
        self.tomato_pose.orientation.z = self.tomato[num_box].pose.pose.orientation.z
        self.tomato_pose.orientation.w = self.tomato[num_box].pose.pose.orientation.w
        
        self.goal_x = self.ar_pose.position.x
        self.goal_y = self.ar_pose.position.y
        self.goal_z = self.ar_pose.position.z
        
        self.position_list = [self.goal_x, self.goal_y, self.goal_z]
        self.orientation_list = [self.ar_pose.orientation.x, self.ar_pose.orientation.y, self.ar_pose.orientation.z, self.ar_pose.orientation.w]
        (self.goal_roll, self.goal_pitch, self.goal_yaw) = euler_from_quaternion(self.orientation_list) 
        
        




        
        '''
        if num_box>0:
            b         = msg.bounding_boxes[0]
            bbox.xmin = b.xmin
            bbox.xmax = b.xmax
            bbox.ymin = b.ymin
            bbox.ymax = b.ymax
        '''
        
if __name__ == '__main__':
    tm = TestMove()
    
    rate = rospy.Rate(10.0)

    tm.pose_subscriber()
    time.sleep(1)

    tm.go_to_move()

    '''
    linear_path = [+0.2, -0.2, +0.2]
    print(">> start path planning")
    cartesian_plan, fraction = tm.plan_cartesian_path(linear_path[0],linear_path[1],linear_path[2])
    tm.display_trajectory(cartesian_plan)
    time.sleep(0.5)
    tm.execute_plan(cartesian_plan)
    '''
    
    print (">> Press the Enter key <<")
    raw_input()
    tm.move_code()  # go to initial pose