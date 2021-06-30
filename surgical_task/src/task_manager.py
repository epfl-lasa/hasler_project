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
import datetime
import os.path
from os import path
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

    self.stopTime = False



    self.image = [];

    rospack = rospkg.RosPack()
    rospack.list() 
    
    self.fileName = fileName
    if rospy.has_param('SurgicalTask/fileName'):
      self.fileName = rospy.get_param("SurgicalTask/fileName")
    else:
      print("[TaskManager]: FileName does not exist on parameter server")

    if rospy.has_param('SurgicalTask/taskId'):
      self.taskId = rospy.get_param("SurgicalTask/taskId")
    else:
      print("[TaskManager]: TaskId does not exist on parameter server")

    self.taskCondition = 0
    if rospy.has_param('SurgicalTask/taskCondition'):
       self.taskCondition = rospy.get_param("SurgicalTask/taskCondition")
    else:
      print("[TaskManager]: Task condition does not exist on parameter server")   

    self.repetitionId = 0
    if rospy.has_param('SurgicalTask/repetitionId'):
       self.repetitionId = rospy.get_param("SurgicalTask/repetitionId")
    else:
      print("[TaskManager]: Repetition does not exist on parameter server")   

    folderPath = rospack.get_path('surgical_task')+"/data/"+self.fileName+"/"
    if not path.isdir(folderPath):
      os.mkdir(folderPath)

    self.file = open(folderPath+self.fileName+"_"+str(self.taskId)+"_"+str(self.taskCondition)+"_"+str(self.repetitionId)+"_task_manager.txt", "w")

      # print("[TaskManager]: Folder path does not exist")   
      # sys.exit()

    self.imagesPath = [rospack.get_path('surgical_task')+"/images/gripper/pick_and_place/",
                       rospack.get_path('surgical_task')+"/images/gripper/gripper_shapes/",
                       rospack.get_path('surgical_task')+"/images/camera/",
                       rospack.get_path('surgical_task')+"/images/gripper/pick_and_place/",
                       rospack.get_path('surgical_task')+"/images/4hands/",
                       rospack.get_path('surgical_task')+"/images/gripper_4_hands_by3/"]

    self.imagesName = ["pick_and_place_",
                       "shapes_",
                       "cam_task_",
                       "pick_and_place_",
                       "4_hands_shoelace",
                       "gripper_4_hands_by3_",]

    self.taskName = ["Gripper Pick and Place",
                     "Gripper Rubber Band",
                     "Camera",
                     "Gripper Camera",
                     "Four Hands Lace",
                     "Four Hands Pick and Place"]


    self.nbImages = [8,5,9,5,1,4]
    self.maxDuration = [1000,300,300,1000,600,600]
    self.imageOrder = []

    self.timeInit = time.time()
    self.t0 = self.timeInit
    self.t = self.t0
    self.tStop = self.t0
    self.tOffset = 0
    self.timeList = []
    self.positionErrorList = []
    self.angleErrorList = []
    self.cameraCueSizeList = []
    self.nbFalls = 0
    self.timeStartText = time.time()
    self.showStartText = True
    self.startText = "Press 's' to start !"
    self.updateTarget = True
    self.cameraWaitTime = 10.0
    self.fullScreen = True
    self.imageId = -1
    self.wait = False

    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

    self.pubState = rospy.Publisher('task_manager/state',TaskManagerStateMsg, queue_size=1)

    self.subMarkerPose = rospy.Subscriber('camera_manager/markers_pose', Float64MultiArray, self.updateMarkersPose)
    self.subCameraCueSize = rospy.Subscriber('camera_manager/camera_cue_size', Float64, self.updateCameraCueSize)
    self.subSurgicalTaskState = rospy.Subscriber('surgical_task/state', SurgicalTaskStateMsg, self.updateSurgicalTaskState)


    self.run()

  def run(self):
    while not rospy.is_shutdown():
      c = cv2.waitKey(1) % 256

      if c == ord('s') and not self.start:
        self.start = True
        self.timeInit = time.time()
        self.t0 = self.timeInit
      elif c == ord('q'):
        self.finished = True

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
        # self.fourHandsSuturing()
        self.fourHandsPickAndPlace(c)


      image = self.image.copy()
      height = image.shape[0]
      width = image.shape[1]

      if not self.stopTime and not self.wait:
        self.t = time.time()-self.tOffset


      if not self.start:
        if self.t-self.timeStartText> 0.5:
          self.showStartText = not self.showStartText
          self.timeStartText = self.t
        if self.showStartText:        
          cv2.putText(image, self.startText, (int(width/2-450),int(height/2)), cv2.FONT_HERSHEY_TRIPLEX, 3, (255,0,255), 2)

      if (self.taskId == 2 or self.taskId == 5) and self.start:
        text = "Tool to reach: " + self.toolName[self.toolToReach]
        # cv2.putText(image, text, (30,height-30), cv2.FONT_HERSHEY_TRIPLEX, 1.2, (255,0,255), 2)
        cv2.putText(image, text, (int(width/2-450),height-20), cv2.FONT_HERSHEY_TRIPLEX, 3, (255,0,255), 2)
      elif (self.taskId == 0 or self.taskId == 3 or self.taskId == 5)and self.start and self.stopTime:
        text = "Recover cylinder"
        cv2.putText(image, text, (int(width/2-450),height-20), cv2.FONT_HERSHEY_TRIPLEX, 3, (255,0,255), 2)
      elif self.taskId == 1 and self.start and self.stopTime:
        text = "Recover rubber band"
        cv2.putText(image, text, (int(width/2-450),height-20), cv2.FONT_HERSHEY_TRIPLEX, 3, (255,0,255), 2)

      remainingTime = self.maxDuration[self.taskId]
      if self.start and not self.wait:
        remainingTime = self.maxDuration[self.taskId]-int(self.t-self.t0)

      # cv2.putText(image, str(datetime.timedelta(seconds=remainingTime)), (50, height-30), cv2.FONT_HERSHEY_TRIPLEX, 2, (255,0,255), 2)
      print(str(datetime.timedelta(seconds=remainingTime)), end="\r") 
      
      if self.start and self.t-self.t0>self.maxDuration[self.taskId]:
          self.finished=True
      
      cv2.namedWindow(self.taskName[self.taskId], cv2.WINDOW_AUTOSIZE)
      if self.fullScreen:
        # imS = cv2.resize(image, (1680, 1050))
        imS = cv2.resize(image, (400, 400))
        # cv2.setWindowProperty(self.taskName[self.taskId], cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        # cv2.moveWindow(self.taskName[self.taskId],4000,0)
        # cv2.moveWindow(self.taskName[self.taskId],2000,0)
        cv2.moveWindow(self.taskName[self.taskId],3440,780)
        cv2.imshow(self.taskName[self.taskId], imS)
      else:
        cv2.imshow(self.taskName[self.taskId], image) 

      msg = TaskManagerStateMsg()

      msg.taskId = self.taskId
      msg.toolToReach = self.toolToReach
      msg.toolReached = self.toolReached
      msg.finished = self.finished
      msg.start = self.start
      msg.stopTime = self.stopTime
      msg.imageId = self.imageId

      self.pubState.publish(msg)

      self.rate.sleep()

      if self.finished:
        print("Total time: ", datetime.timedelta(seconds=int(self.t-self.t0)))
        break
        
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
        if positionError < 30 and angleError < 10:
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
  

  def fourHandsPickAndPlace(self, c):
    if not self.initialized:
      self.initialized = True
      print("FOUR HANDS PICK AND PLACE")

      self.imageId = 1
      print("Image ID: ", self.imageId)

      self.toolToReach = 0

    if self.start:
      self.updateGripperTarget(c)
    
    if self.updateTarget or not self.start:
      path = self.imagesPath[self.taskId]+self.imagesName[self.taskId]+str(self.imageId)+".png"
      self.image = cv2.imread(path)

  def updateGripperTarget(self, c):
    if c == ord('r'):
      if not self.stopTime:
        self.tStop = time.time()
        self.nbFalls = self.nbFalls+1
        self.stopTime = True
        print("Nb falls:", self.nbFalls)
        print(self.timeList)
        print("[TaskManager]: Recover the object")
      else:
        self.stopTime = False
        self.timeInit = time.time()
        self.tOffset = self.tOffset+self.timeInit-self.tStop
        self.t = time.time()-self.tOffset
      self.updateTarget = False

    elif c == ord('n'):
      self.imageId = self.imageId + 1
      self.toolToReach = self.toolToReach + 1
      if self.toolToReach > 2:
        self.toolToReach = 0

      self.timeList.append(time.time()-self.timeInit)
      print(self.timeList)
      self.timeInit = time.time()
      self.updateTarget = True
      if self.imageId > self.nbImages[self.taskId]:
        self.imageId = self.nbImages[-1]
        self.finished=True
      else:
        print("Image ID: ", self.imageId)

    elif c == ord('p'):
      self.imageId = self.imageId - 1
      if len(self.timeList) > 0:
        self.timeList.pop()
      print(self.timeList)
      self.timeInit = time.time()
      print("Image ID: ", self.imageId)
      self.imageId = max(1,min(self.imageId, self.nbImages[self.taskId]))
      self.updateTarget = True
    else:
      self.updateTarget = False


  def logData(self):
    if self.taskId == Task.GRIPPER_PICK_AND_PLACE.value or self.taskId == Task.GRIPPER_RUBBER_BAND.value or self.taskId == Task.GRIPPER_CAMERA.value or self.taskId == Task.FOUR_HANDS_SUTURING.value: 
      self.file.write(str(len(self.timeList)))
      self.file.write("\n")
      for t in self.timeList:
        self.file.write(str("%f " % t))
      self.file.write("\n")
      self.file.write(str(self.nbFalls))
    elif self.taskId == Task.CAMERA.value:
      self.file.write(str("%d " % self.imageId))
      self.file.write("\n")
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
    elif self.taskId == Task.FOUR_HANDS_LACE.value: 
      self.file.write(str(time.time()-self.t0))
    # elif self.taskId == Task.FOUR_HANDS_SUTURING.value:
    #   self.file.write(str("%d " % self.imageId))
    #   self.file.write("\n")
    #   self.file.write(str(time.time()-self.t0))


  def updateMarkersPose(self, msg):
    self.markerPose = np.reshape(msg.data,(3,5))

  def updateCameraCueSize(self, msg):
    self.cameraCueSize = msg.data

  def updateSurgicalTaskState(self,msg):
    if self.wait == True  and msg.wait == False:
      self.wait = msg.wait
      self.timeInit = time.time()
      self.t0 = self.timeInit


if __name__ == '__main__':
  rospy.init_node('task_manager', anonymous=True)

  taskId = 5
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
  elif len(sys.argv) > 1:
    print("Wrong number of input arguments !")
    print("Usage: rosrun surgical_task task_manager fileName taskId")
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