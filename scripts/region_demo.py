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

JOINTS_PRE_DROP = [-1.1517155806170862, -1.2092912954143067, -2.1720776557922363, 4.9934096215064905, 1.5708632469177246, -1.1364582220660608]
JOINTS_DROP = [-1.1516197363482874, -1.2878874105266114, -2.2760825157165527, 5.157104956894674, 1.5586934089660645, -1.1299169699298304]
JOINTS_POST_DROP = [-1.1078723112689417, -1.0811011356166382, -2.4803175926208496, 5.335938203125753, 1.5054988861083984, -1.0957582632647913]

JOINTS_PRE_GRASP = [-1.090461556111471, -1.0595074456981202, -2.3696846961975098, 5.146625268250265, 1.513023853302002, -1.084428612385885]
JOINTS_GRASP = [-1.1078723112689417, -1.0811011356166382, -2.4803175926208496, 5.335938203125753, 1.5054988861083984, -1.0957582632647913]

JOINTS_FRONT_SAFE = [-1.2100423018084925, -0.7609225076488038, -1.1158299446105957, 4.105232878322266, 1.3995943069458008, -1.2187255064593714]
JOINTS_MIDDLE = [-1.680774990712301, -2.2949754200377406, 2.036011521016256, 6.214656102448263, 1.574042797088623, -0.15153009096254522]

JOINT_BACK = [-1.6767361799823206, -0.5919974607280274, 2.0268734137164515, 6.172548997193136, 1.5772318840026855, -0.15136272112001592]








def detect(HOGCV, frame):
  bounding_box_cordinates, weights =  HOGCV.detectMultiScale(frame, winStride = (4, 4), padding = (8, 8), scale = 1.03)

  nb = len(bounding_box_cordinates)

  for (x, y, w, h) in bounding_box_cordinates:
      cv2.rectangle(frame, (x, y), 
                    (x + w, y + h), 
                    (0, 0, 255), 2)  
      
  # cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
  # cv2.resizeWindow('Image', 1920, 1080)  
  # cv2.imshow("Image", frame)



  return nb



def detectByCamera(HOGCV):
       
  capture = cv2.VideoCapture(0)
  # capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
  # width = 1920
  # height = 1080
  # capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  # capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


  print('Detecting people...')



  while True:

    check, frame = capture.read()
    frame = imutils.resize(frame, width = min(800, frame.shape[1])) 
    cv2.namedWindow('Image_raw',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image_raw', 800, 600)      

    


    #cv2.imshow("Image_resized", frame)

    nb = detect(HOGCV, frame)
    print (nb)
    if(nb > 10 or rospy.is_shutdown()):
      break
    #sleep(1)
    cv2.waitKey(1)
    cv2.imshow("Image_raw", frame)

  cv2.destroyAllWindows()  
  capture.release()
  


  



def detectNobodyByCamera(HOGCV):   
  video = cv2.VideoCapture(0)
  print('Detecting if nobody')

  while True:
    check, frame = video.read()
    

    nb = detect(HOGCV, frame)
    print (nb)
    if(nb < 1 or rospy.is_shutdown()):
      break
    sleep(1)


  video.release()
  cv2.destroyAllWindows()    



def detect_group(faceCascade, mode):

  

  while True:
    video = cv2.VideoCapture(0)
    check, image = video.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),

    )    

    print "Found {0} faces!".format(len(faces))
    i=0
    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
      i=i+1
      cv2.rectangle(image, (x, y), (x+w, y+h), (27*i%256, 255-27*i%256, 27*i%256), 2)

    cv2.imshow("Faces found" ,image)
    cv2.waitKey(500)
      
    print (i)
    if (i > 1 and mode == 1) or (i < 1 and mode == 0):
      break
    if rospy.is_shutdown():
      quit()
    sleep(1)

    video.release()
  cv2.destroyAllWindows()   




if __name__ == '__main__':
  rospy.init_node('lyontech_arm_commander', anonymous=True)

  p = os.path.dirname( os.path.abspath( os.path.realpath(__file__) ) )
  print ( p )

  faceCascade = cv2.CascadeClassifier(p + '/face.xml')

  #capture = cv2.VideoCapture(0)

  #HOGCV = cv2.HOGDescriptor()
  #HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

  


  #detectByCamera(HOGCV)
  #quit()

  gripper = Gripper()
  mg = MoveGroup_Palbator_UR5e()

  mg.go_to_up_state()

  #mg.add_box()


  # engine = pyttsx3.init()

  # rate = engine.getProperty('rate')   # getting details of current speaking rate
  # print (rate)                        #printing current voice rate
  # engine.setProperty('rate', 80)     # setting up new voice rate  

  # volume = engine.getProperty('volume')   #getting to know current volume level (min=0 and max=1)
  # print (volume)                          #printing current volume level
  # engine.setProperty('volume',1.0)    # setting up volume level  between 0 and 1

  # voices = engine.getProperty('voices') # the french voice
  # for i,voice in enumerate(voices):
  #   print (i, ": ", voice.__dict__)
  #   if voice.id == u"french":
  #     break

  # engine.setProperty('voice', voice.id)


  while not rospy.is_shutdown():

    gripper.open()

    mg.group.go(JOINTS_PRE_GRASP, wait=True)
    mg.group.go(JOINTS_GRASP, wait=True)

    gripper.close(100)

    mg.group.go(JOINTS_PRE_GRASP, wait=True)
    mg.group.go(JOINTS_PRE_DROP, wait=True)
    mg.group.go(JOINTS_FRONT_SAFE, wait=True)
    mg.group.go(JOINTS_MIDDLE, wait=True)
    mg.group.go(JOINT_BACK, wait=True)

    # Detection
    detect_group(faceCascade, 1)
    #detectByCamera(HOGCV)

    #engine.say('Oh... la region est la !') # perfect  
    #engine.runAndWait()
    playsound.playsound(p + '/synthesize.mp3', True)

    mg.group.go(JOINTS_MIDDLE, wait=True)
    mg.group.go(JOINTS_FRONT_SAFE, wait=True)

    swing = deepcopy(JOINTS_FRONT_SAFE)
    swing[5] -= 0.5
    mg.group.go(swing, wait=True)
    swing[5] += 1
    mg.group.go(swing, wait=True)

    mg.group.go(JOINTS_PRE_DROP, wait=True)
    mg.group.go(JOINTS_DROP, wait=True)
    mg.group.go(JOINTS_POST_DROP, wait=True)
    gripper.open()
    mg.group.go(JOINTS_PRE_GRASP, wait=True)

    # Detect nobody before loop
    detect_group(faceCascade, 0)
    #detectNobodyByCamera(HOGCV)


  



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



  mg.remove_world_object.remove_box("socle")
  mg.remove_world_object.remove_box("front")




