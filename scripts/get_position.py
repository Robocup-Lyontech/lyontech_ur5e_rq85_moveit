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

if __name__ == '__main__':
  rospy.init_node('lyontech_arm_commander', anonymous=True)

  mg = MoveGroup_Palbator_UR5e()

  while not rospy.is_shutdown():

    raw_input("Press Enter to Go Home")

    current_joints = mg.group.get_current_joint_values()

    print( current_joints )








