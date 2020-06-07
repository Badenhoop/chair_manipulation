#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planning',
                anonymous=True)

robot = moveit_commander.RobotCommander()
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)
group.set_named_target('up')
group.go()
group.stop()
group.clear_pose_targets()