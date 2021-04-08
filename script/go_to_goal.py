#!/usr/bin/env python

import sys
import rospy
import tf
import moveit_commander
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move', anonymous=True)
group = [moveit_commander.MoveGroupCommander("manipulator")]

count = 1

while not rospy.is_shutdown():
    if(count%2 == 1):
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.15
        pose_goal.position.z = 0.2
        group[0].set_pose_target(pose_goal)
        group[0].go(True)

        print("Count", count ,pose_goal)

        rospy.sleep(2)
    
    else:
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.15
        pose_goal.position.z = 0.3
        group[0].set_pose_target(pose_goal)
        group[0].go(True)

        print("Count", count, pose_goal)

        rospy.sleep(2)

    count += 1

moveit_commander.roscpp_shutdown()