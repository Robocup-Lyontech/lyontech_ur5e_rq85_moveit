#!/usr/bin/env python

#
# Code adapted from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# to UR5 robot
#
from math import pi
import os
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
from Arm_Env import MoveGroup_Palbator_UR5e_Env
from Gripper import Gripper
from copy import deepcopy
from time import sleep
import playsound

import cv2
import imutils
import numpy as np
import argparse

import pyttsx3

## END_SUB_TUTORIAL

UR5_ZERO = [0.0, -1.5746318302550257, 0.004732433949605763, -1.5784551105894984, 0.0, 0.0]

UR5_HOME = [1.5914058685302734, -1.618694921533102, 2.821789566670553, -1.0671308797648926, 3.160884141921997, 2.11466121673584]

UR5_GROUND_PREGRASP = [2.464851204548971, -1.2727511686137696, 1.1997385025024414, 3.421120806331299, 4.731529712677002, -0.3448084036456507]

UR5_SWIFT_BEGIN = [1.896212402974264, -0.01641757905993657, 2.2380855083465576, 4.385730429286621, 1.7052478790283203, -3.1222007910357874]

UR5_SWIFT_WIDE_BEGIN = [1.3198631445514124, -0.04147584856066899, -0.23618823686708623, 4.982526051789083, 4.531479358673096, -3.1214564482318323]
UR5_SWIFT_WIDE_END = [1.3197792212115687, -0.04314811647448735, 2.455706834793091, 4.982369887619772, 4.531479358673096, -3.1214922110186976]


UR5_BAG_PREBAG = [0.9188055992126465, -0.5067761701396485, 0.5335648695575159, -1.559631661777832, 4.703949928283691, 1.2541351318359375]
UR5_BAG_GRASPBAG = [0.930718183517456, -0.45122583330187993, 0.7720721403705042, -1.9027735195555628, 4.652039527893066, 1.2540631294250488]

UR5_BAG_TRANSP = [-0.22848111787904912, -2.039434095422262, 1.6872642675982874, -1.00609643877063, 4.5479583740234375, 0.9501953125]




if __name__ == '__main__':
  rospy.init_node('lyontech_arm_commander', anonymous=True)


  gripper = Gripper()
  mg = MoveGroup_Palbator_UR5e()
  sleep(3)
  mg.add_palbator_scene()


  #mg.go_to_up_state()

  #mg.add_box()


 # while not rospy.is_shutdown():

  gripper.close(100)

  #mg.go_to_up_state()

  a = raw_input("Press Enter to Go Home")


  mg.group.go(UR5_HOME, wait=True)

  a = raw_input("Press Enter to Pre grasp")

  mg.group.go(UR5_BAG_PREBAG, wait=True)

  gripper.open()

  a = raw_input("Press Enter to Grasp")

  mg.group.go(UR5_BAG_GRASPBAG, wait=True)

  gripper.close(100)

  mg.group.go(UR5_BAG_PREBAG, wait=True)

  a = raw_input("Press Enter to transp")

  mg.group.go(UR5_BAG_TRANSP, wait=True)

  mg.remove_box("Back")
  mg.remove_box("Top")
  mg.remove_box("Base")
  mg.remove_box("Equip")
  mg.scene.remove_world_object()
  mg.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4)

  sleep(5)


  # p = deepcopy(mg.POSE_STD)
  # p.orientation = mg.ORI_DOWN

  # p.position.z = p.position.z 
  # mg.go_to_pose_goal(p)
  # p.position.z = p.position.z - 0.7
  # mg.go_to_pose_goal(p)

  # p.position.z = p.position.z - 0.13
  # mg.go_to_pose_goal(p)

  # gripper.close(35)

  # p.position.z = p.position.z 
  # mg.go_to_pose_goal(p)

  # p = deepcopy(mg.POSE_STD)
  # p.position.z = p.position.z + 0.03
  # mg.go_to_pose_goal(p)

  # p.position.y = p.position.y + 0.35
  # mg.go_to_pose_goal(p)

  # gripper.open()

  # p.position.y = p.position.y - 0.35
  # mg.go_to_pose_goal(p)

  current_joints = mg.group.get_current_joint_values()

  print( current_joints )








