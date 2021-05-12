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
import rospkg
import sys
# import keyboard



class Task(Enum):
  GRIPPER_PICK_AND_PLACE = 0
  GRIPPER_RUBBER_BAND = 1
  CAMERA = 2
  GRIPPER_CAMERA = 3
  FOUR_HANDS_LACE = 4
  FOUR_HANDS_SUTURING = 5

class TaskManager:
  def __init__(self, taskId):

    self.rate = rospy.Rate(200) 
    
    self.pubState = rospy.Publisher('task_manager/state',TaskManagerStateMsg, queue_size=1)

    self.subMarkerPose = rospy.Subscriber('camera_manager/markers_pose', Float64MultiArray, self.updateMarkersPose)

    self.taskId = taskId
    self.toolToReach = -1

    self.initialized = False
    self.cameraCount = 0
    self.toolReached = False
    self.timeReached = time.time()

    self.markerPose = np.zeros((3,5))

    self.toolName = ['R', 'G', 'Y']

    self.toolErrorthreshold = 30

    self.finished = False


    self.image = [];

    rospack = rospkg.RosPack()
    rospack.list() 
    
    self.imagesPath = [rospack.get_path('surgical_task')+"/images/gripper/pick_and_place/",
                       rospack.get_path('surgical_task')+"/images/gripper/gripper_shapes/",
                       rospack.get_path('surgical_task')+"/images/camera/",
                       rospack.get_path('surgical_task')+"/images/gripper/pick_and_place/",
                       rospack.get_path('surgical_task')+"/images/4hands/",
                       rospack.get_path('surgical_task')+"/images/4hands/stitches/"]

    self.imagesName = ["pick_and_place_",
                       "shapes_",
                       "camera_task_",
                       "pick_and_place_",
                       "4_hands_shoelace",
                       "4_hands_stiches_",]

    self.taskName = ["Gripper Pick and Place",
                     "Gripper Rubber Band",
                     "Camera",
                     "Gripper Camera",
                     "Four Hands Lace",
                     "Four Hands Suturing"]


    self.nbImages = [14,9,26,9,1,9]
    self.imageOrder = []

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

      cv2.imshow(self.taskName[self.taskId], self.image) 

      c = cv2.waitKey(1) % 256

      if c == ord('n') and self.taskId == Task.GRIPPER_PICK_AND_PLACE.value:
        self.imageId = self.imageId + 1
        if self.imageId > self.nbImages[self.taskId]:
          break
        print("Image ID: ", self.imageId)

      elif c == ord('p') and self.taskId == Task.GRIPPER_PICK_AND_PLACE.value:
        self.imageId = self.imageId - 1
        print("Image ID: ", self.imageId)
        self.imageId = max(1,min(self.imageId, self.nbImages[self.taskId]))
      
      elif c== ord('q'):
        break


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

      self.imageId = 1
      print("Image ID: ", self.imageId)


    path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
    self.image = cv2.imread(path)


  def gripperRubberBand(self):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER RUBBER BAND")

      self.imageId = random.randint(1,self.nbImages[self.taskId])
      print("Image ID: ", self.imageId)
      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
      self.image = cv2.imread(path)


  def camera(self):
    if not self.initialized:
      self.initialized = True
      print("CAMERA")
      self.imageId = random.randint(1,self.nbImages[self.taskId])
      print("Image ID: ", self.imageId)
      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
      self.image = cv2.imread(path)
     
      self.toolOrder = [0,1,2]
      random.shuffle(self.toolOrder)
      sequenceOrder = [self.toolName[idx] for idx in self.toolOrder]
      print("Tool order: ", sequenceOrder)
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

      self.imageId = 1
      print("Image ID: ", self.imageId)


    path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
    self.image = cv2.imread(path)


  def fourHandsLace(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS LACE")

      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+".png"
      self.image = cv2.imread(path)
  

  def fourHandsSuturing(self):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS SUTURING")

      self.imageId = random.randint(1,self.nbImages[self.taskId])
      print("Image ID: ", self.imageId)
      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
      self.image = cv2.imread(path)
  

  def updateMarkersPose(self, msg):
    self.markerPose = np.reshape(msg.data,(3,5))


if __name__ == '__main__':
  rospy.init_node('task_manager', anonymous=True)

  taskId = 0

  if len(sys.argv) == 2:
    if int(sys.argv[1]) >= 0 and int(sys.argv[1])<=5:
      taskId = int(sys.argv[1])
    else:
      print("Task ID shoud be between 0 and 5:")
      print("0: Gripper Pick and Place")
      print("1: Gripper Rubber Band")
      print("2: Camera")
      print("3: Gripper Camera")
      print("4: Four Hands Lace")
      print("5: Four Hands Suturing")
      sys.exit(0)
  else:
    print("bou")

  taskManager = TaskManager(taskId)
  cv2.destroyAllWindows()    