#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from surgical_task.msg import SurgicalTaskStateMsg, RobotStateMsg
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
    
    self.pubTaskId = rospy.Publisher('task_manager/task_id',Int16, queue_size=1)

    self.taskId = 2

    self.initialized = False

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

      msg = Int16()

      msg.data = self.taskId

      self.pubTaskId.publish(msg)

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
      print(self.toolOrder)

      
  def gripperCamera(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS LACE")

  def fourHandsLace(self):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER PICK AND PLACE")

  def fourHandsSuturing(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS SUTURING")


if __name__ == '__main__':
  rospy.init_node('task_manager', anonymous=True)
  taskManager = TaskManager()
  cv2.destroyAllWindows()    