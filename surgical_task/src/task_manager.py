#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from surgical_task.msg import SurgicalTaskStateMsg, RobotStateMsg, TaskManagerStateMsg
import time
import rospkg
from enum import Enum
from random import randrange, shuffle
import random



class Task(Enum):
  GRIPPER_PICK_AND_PLACE = 0
  GRIPPER_RUBBER_BAND = 1
  CAMERA = 2
  GRIPPER_CAMERA = 3
  FOUR_HANDS_LACE = 4
  FOUR_HANDS_SUTURING = 5

class TaskManager:
  def __init__(self):

    self.rate = rospy.Rate(200) 
    
    self.pubState = rospy.Publisher('task_manager/state',TaskManagerStateMsg, queue_size=1)

    self.subMarkerPose = rospy.Subscriber('camera_manager/markers_pose', Float64MultiArray, self.updateMarkersPose)

    self.taskId = 2
    self.toolToReach = -1

    self.initialized = False
    self.cameraCount = 0
    self.toolReached = False
    self.timeReached = time.time()

    self.markerPose = np.zeros((3,5))

    self.toolName = ['R', 'G', 'Y']

    self.toolErrorthreshold = 30

    self.finished = False

    self.run()


  def run(self):
    while not rospy.is_shutdown():
      t0 = time.time()
      if self.taskId == Task.GRIPPER_PICK_AND_PLACE.value:
        self.gripperPickAndPlace()
      elif self.taskId == Task.GRIPPER_RUBBER_BAND.value:
        self.gripperRubberBand()
      elif self.taskId == Task.CAMERA.value:
        self.camera()
      elif self.taskId == Task.GRIPPER_CAMERA.value:
        self.gripperCamera()
      elif self.taskId == Task.FOUR_HANDS_LACE.value:
        self.fourHandsLace()
      elif self.taskId == Task.FOUR_HANDS_SUTURING.value:
        self.fourHandsSuturing()

      msg = TaskManagerStateMsg()

      msg.taskId = self.taskId
      msg.toolToReach = self.toolToReach
      msg.toolReached = self.toolReached

      self.pubState.publish(msg)

      self.rate.sleep()


  def gripperPickAndPlace(self):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER PICK AND PLACE")


  def gripperRubberBand(self):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER RUBBER BAND")

  def camera(self):
    if not self.initialized:
      self.initialized = True
      print("CAMERA")
      self.toolOrder = [0,1,2]
      random.shuffle(self.toolOrder)
      print("Tool order: ", self.toolOrder)
      self.toolToReach = self.toolOrder[self.cameraCount]
      print("Tool to track: ", self.toolName[self.toolToReach])

    if (self.markerPose[self.toolToReach][2]) and not self.finished:
      error = np.linalg.norm(self.markerPose[self.toolToReach][0:2]-np.array([640/2,480/2]))
      if not self.toolReached:
        if error < 30:
          self.toolReached = True
          self.timeReached = time.time()
        # print(self.markerPose[self.toolToReach][0:2], error)
      else:
        print(time.time()-self.timeReached)
        if (time.time()-self.timeReached)>3:
          print(error)
          self.toolReached = False
          self.cameraCount = self.cameraCount+1
          if self.cameraCount > 2:
            self.finished = True
            print("Finished")
          else:
            self.toolToReach = self.toolOrder[self.cameraCount]
            print("Tool to track: ", self.toolName[self.toolToReach])



      
  def gripperCamera(self):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER CAMERA")

  def fourHandsLace(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS LACE")

  def fourHandsSuturing(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS SUTURING")



  def updateMarkersPose(self, msg):
    self.markerPose = np.reshape(msg.data,(3,5))


if __name__ == '__main__':
  rospy.init_node('task_manager', anonymous=True)
  taskManager = TaskManager()
  cv2.destroyAllWindows()    