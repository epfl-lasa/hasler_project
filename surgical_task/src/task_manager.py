#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension, Int16, Float64
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
  def __init__(self, fileName, taskId):

    self.rate = rospy.Rate(40) 
    
    self.pubState = rospy.Publisher('task_manager/state',TaskManagerStateMsg, queue_size=1)

    self.subMarkerPose = rospy.Subscriber('camera_manager/markers_pose', Float64MultiArray, self.updateMarkersPose)
    self.subCameraCueSize = rospy.Subscriber('camera_manager/camera_cue_size', Float64, self.updateCameraCueSize)

    self.taskId = taskId
    self.toolToReach = -1

    self.initialized = False
    self.cameraCount = 0
    self.toolReached = False

    self.markerPose = np.zeros((3,5),dtype=float)

    self.toolName = ['R', 'G', 'Y']

    self.toolErrorthreshold = 30

    self.finished = False

    self.start = False

    self.cameraCueSize = 0.0


    self.image = [];

    rospack = rospkg.RosPack()
    rospack.list() 
    
    self.fileName = fileName

    self.file = open(rospack.get_path('surgical_task')+"/data/task_manager_"+str(self.taskId)+"_"+self.fileName+".txt", "w")

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


    self.nbImages = [15,7,26,15,1,9]
    self.imageOrder = []

    self.timeInit = time.time()
    self.timeList = []
    self.positionErrorList = []
    self.angleErrorList = []
    self.cameraCueSizeList = []
    self.nbFalls = 0
    self.timeStartText = time.time()
    self.showStartText = True
    self.startText = "Press 's' to start !"
    self.updateTarget = True
    self.cameraWaitTime = 5.0

    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

    self.run()

  def run(self):
    while not rospy.is_shutdown():
      c = cv2.waitKey(1) % 256

      if c == ord('s') and not self.start:
        self.start = True
        self.timeInit = time.time()
        self.t0 = self.timeInit
      elif c == ord('q'):
        break

      if self.taskId == Task.GRIPPER_PICK_AND_PLACE.value:
        self.gripperPickAndPlace(c)
      elif self.taskId == Task.GRIPPER_RUBBER_BAND.value:
        self.gripperRubberBand(c)
      elif self.taskId == Task.CAMERA.value:
        self.camera()
      elif self.taskId == Task.GRIPPER_CAMERA.value:
        self.gripperCamera(c)
      elif self.taskId == Task.FOUR_HANDS_LACE.value:
        self.fourHandsLace()
      elif self.taskId == Task.FOUR_HANDS_SUTURING.value:
        self.fourHandsSuturing()

      if self.finished:
        break

      image = self.image.copy()
      if not self.start:
        if time.time()-self.timeStartText> 0.5:
          self.showStartText = not self.showStartText
          self.timeStartText = time.time()
        if self.showStartText:        
          height = image.shape[0]
          width = image.shape[1]
          cv2.putText(image, self.startText, (int(width/2-300),int(height/2)), cv2.FONT_HERSHEY_TRIPLEX, 2, (255,0,255), 2)

      cv2.imshow(self.taskName[self.taskId], image) 

      msg = TaskManagerStateMsg()

      msg.taskId = self.taskId
      msg.toolToReach = self.toolToReach
      msg.toolReached = self.toolReached

      self.pubState.publish(msg)

      self.rate.sleep()

    self.logData()

    self.file.close()


  def gripperPickAndPlace(self, c):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER PICK AND PLACE")

      self.imageId = 1
      print("Image ID: ", self.imageId)

    if self.start:
      self.updateGripperTarget(c)
    
    if self.updateTarget or not self.start:
      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
      self.image = cv2.imread(path)


  def gripperRubberBand(self, c):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER RUBBER BAND")

      self.imageId = 1
      print("Image ID: ", self.imageId)

    if self.start:
      self.updateGripperTarget(c)
    
    if self.updateTarget or not self.start:
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
      
    if (self.markerPose[self.toolToReach][2]) and not self.finished and self.start:
      positionError = np.linalg.norm(self.markerPose[self.toolToReach][0:2]-np.array([640/2,480/2]))
      angleError = np.arccos(np.dot(self.markerPose[self.toolToReach][3:5], np.array([0.0,-1.0])))*180.0/np.pi
      if not self.toolReached:
        if positionError < 30:
          print(self.markerPose[self.toolToReach][0:2], positionError)
          self.toolReached = True
          self.timeList.append(time.time()-self.timeInit)
          self.timeInit = time.time()

      else:
        if (time.time()-self.timeInit)>self.cameraWaitTime:
          self.timeInit = time.time()
          print(positionError, angleError)
          self.positionErrorList.append(positionError)
          self.angleErrorList.append(angleError)
          self.cameraCueSizeList.append(self.cameraCueSize)
          self.toolReached = False
          self.cameraCount = self.cameraCount+1
          if self.cameraCount > 2:
            self.finished = True
            print("Finished")
          else:
            self.toolToReach = self.toolOrder[self.cameraCount]
            print("Tool to track: ", self.toolName[self.toolToReach])


  def gripperCamera(self, c):
    if not self.initialized:
      self.initialized = True
      print("GRIPPER CAMERA")

      self.imageId = 1
      print("Image ID: ", self.imageId)

    if self.start:
      self.updateGripperTarget(c)

    if self.updateTarget or not self.start:
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
  

  def updateGripperTarget(self, c):
    if c == ord('r'):
      self.nbFalls = self.nbFalls+1
      print(self.nbFalls)
      print(self.timeList)
      self.timeInit = time.time()
      self.updateTarget = True

    elif c == ord('n'):
      self.imageId = self.imageId + 1
      self.timeList.append(time.time()-self.timeInit)
      print(self.timeList)
      self.timeInit = time.time()
      self.updateTarget = True
      if self.imageId > self.nbImages[self.taskId]:
        self.finished=True
      else:
        print("Image ID: ", self.imageId)

    elif c == ord('p'):
      self.imageId = self.imageId - 1
      self.timeList.pop()
      print(self.timeList)
      self.timeInit = time.time()
      print("Image ID: ", self.imageId)
      self.imageId = max(1,min(self.imageId, self.nbImages[self.taskId]))
    else:
      self.updateTarget = False


  def logData(self):
    if self.taskId == Task.GRIPPER_PICK_AND_PLACE.value or self.taskId == Task.GRIPPER_RUBBER_BAND.value or self.taskId == Task.GRIPPER_CAMERA.value: 
      self.file.write(str(len(self.timeList)))
      self.file.write("\n")
      for t in self.timeList:
        self.file.write(str("%f " % t))
      self.file.write("\n")
      self.file.write(str(self.nbFalls))
    elif self.taskId == Task.CAMERA.value:
      for t in self.timeList:
        self.file.write(str("%f " % t))
      self.file.write("\n")
      for p in self.positionErrorList:
        self.file.write(str("%f " % p))
      self.file.write("\n")
      for a in self.angleErrorList:
        self.file.write(str("%f " % a))
      self.file.write("\n")
      for c in self.cameraCueSizeList:
        self.file.write(str("%f " % c))      
    elif self.taskId == Task.FOUR_HANDS_LACE.value or self.taskId == Task.FOUR_HANDS_SUTURING.value:
      self.file.write(str(time.time()-self.t0))


  def updateMarkersPose(self, msg):
    self.markerPose = np.reshape(msg.data,(3,5))

  def updateCameraCueSize(self, msg):
    self.cameraCueSize = msg.data


if __name__ == '__main__':
  rospy.init_node('task_manager', anonymous=True)

  taskId = 0
  fileName = "test"

  if len(sys.argv) == 3:
    fileName = sys.argv[1]
    if int(sys.argv[2]) >= 0 and int(sys.argv[2])<=5:
      taskId = int(sys.argv[2])
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
    print("Wrong number of input arguments !")
    print("Usage: rosrun surgical_task task_manager fileName taskID")
    print("TaskID shoud be between 0 and 5:")
    print("0: Gripper Pick and Place")
    print("1: Gripper Rubber Band")
    print("2: Camera")
    print("3: Gripper Camera")
    print("4: Four Hands Lace")
    print("5: Four Hands Suturing")
    sys.exit(0)

  taskManager = TaskManager(fileName,taskId)
  cv2.destroyAllWindows()    