#!/usr/bin/env python

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
import moveit_commander

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from std_srvs.srv import Empty

from math import *
try:
    from math import tau
except:
    from math import pi
    tau = 2.0*pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    # d = dist((x1, y1, z1), (x0, y0, z0))
    a = np.array((x1, y1, z1))
    b = np.array((x0, y0, z0))
    d = np.linalg.norm(a-b)
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True

class MoveGroupPythonInterfaceUR(object):
    def __init__(self):
        super(MoveGroupPythonInterfaceUR, self).__init__()

        ## initialize 'moveit_commander', 'rospy' node:
        roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_ur', anonymous=True)

        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## surrounding world
        scene = moveit_commander.PlanningSceneInterface()

        ## set the ur robot arm group's name to "ur3_arm"
        ## This interface can be used to plan and execute motions

        move_group = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)

        ## create a `DisplayTrajectory`
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                        moveit_msgs.msg.DisplayTrajectory, 
                                                        queue_size=20)

        planning_frame = move_group.get_planning_frame()
        print("=========== Planning frame : %s" % planning_frame)

        ee_link = move_group.get_end_effector_link()
        print("=========== End effector link: %s" % ee_link)

        group_names = robot.get_group_names()
        print("=========== Available Planning Groups: ", robot.get_group_names())

        print("=========== Printing robot state")
        print(robot.get_current_state())
        print("")
                                    
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau/8
        joint_goal[2] = 0
        joint_goal[3] = -tau/4
        joint_goal[4] = 0
        joint_goal[5] = tau/6  # 1/6 of a turn

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        move_group = self.move_group

        # planning to a pose goal
        # plan a motion for this group to a desiered pose for the end-ffector
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        
        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # cartesian path planning
        # speicify a list of waypoints for the end-effector to go through

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,      # waypoints to follow
                                        0.01,           # step
                                        0.0)             # jump_threshold

        # just planning, not let move_group to actually move the robot yet
        return plan, fraction                                    

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # display a trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()                                           
        display_trajectory.trajectory_start = robot.get_current_state() 
        display_trajectory.trajectory.append(plan)
        # publish
        display_trajectory_publisher.publish(display_trajectory);
    
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
        tutorial = MoveGroupPythonInterfaceUR()

        print("============ Press `Enter` to execute a movement using a joint state goal ...")
        tutorial.go_to_joint_state()

        print("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()

        print("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        print("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
        tutorial.display_trajectory(cartesian_plan)

        print("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)

        print("============ Python tutorial demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__=='__main__':
    main()