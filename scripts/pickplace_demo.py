#!/usr/bin/env python

#
# Code adapted from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# to UR5 robot
#
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander.conversions import pose_to_list
from Arm import MoveGroup_Palbator_UR5e
from Gripper import Gripper
from copy import deepcopy
## END_SUB_TUTORIAL



if __name__ == '__main__':
  rospy.init_node('lyontech_arm_commander', anonymous=True)
  gripper = Gripper()
  mg = MoveGroup_Palbator_UR5e()
  mg.go_to_up_state()

  mg.add_box()

  p = deepcopy(mg.POSE_STD)
  p.orientation = mg.ORI_DOWN

  p.position.z = p.position.z 
  mg.go_to_pose_goal(p)
  p.position.z = p.position.z - 0.7
  mg.go_to_pose_goal(p)

  p.position.z = p.position.z - 0.13
  mg.go_to_pose_goal(p)

  gripper.close(35)

  p.position.z = p.position.z 
  mg.go_to_pose_goal(p)

  p = deepcopy(mg.POSE_STD)
  p.position.z = p.position.z + 0.03
  mg.go_to_pose_goal(p)

  p.position.y = p.position.y + 0.35
  mg.go_to_pose_goal(p)

  gripper.open()

  p.position.y = p.position.y - 0.35
  mg.go_to_pose_goal(p)



  mg.remove_world_object.remove_box("box")
  mg.remove_world_object.remove_box("box2")




