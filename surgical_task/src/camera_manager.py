#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from surgical_task.msg import SurgicalTaskStateMsg


class CameraManager:
  def __init__(self):

    self. image = []
    self.inputImage = []
    self.outputImage = []
    self.rate = rospy.Rate(30) 

    self.useRosCvCamera = rospy.get_param("useRosCvCamera")
    if self.useRosCvCamera:
      self.firstImage = False
      self.bridge = CvBridge()
      self.subCamera = rospy.Subscriber(rospy.get_param("cameraTopic"), Image, self.updateImage)
    else:
      self.cap = cv2.VideoCapture(rospy.get_param("cameraPort"))

    self.pubMarkersPositionTransformed = rospy.Publisher('surgical_task/markers_position_transformed', Float64MultiArray, 
                                                         queue_size=1)
    self.pubMarkersPosition = rospy.Publisher('endoscope_modifier/markers_position', Int16MultiArray, queue_size=1)
    self.subSurgicalTaskState = rospy.Subscriber('surgical_task/state', SurgicalTaskStateMsg, self.updateSurgicalTaskState)

    cv2.namedWindow("output", cv2.WINDOW_NORMAL)  

    self.toolsTracker = ToolsTracker()

    self.humanInputMode = 0 
    self.controlPhase = [0, 0]

    self.robotTool = ["Camera: ", "Tool: "]
    self.robotColor = [(0,0,255), (255,0,0)]

    self.controlPhaseText = ["INSERTION","OPERATION"]
    self.controlPhaseTextPosition = [(20,30),
                                     (440,30)]

    self.cameraModeText = ["Joystick","Assistance"]
    self.cameraModeTextPosition = (20,60)

    self.clutchingStateText = ["Off", "On"]
    self.clutchingStateTextPosition = (440,60)

    self.markerText = ["R","G"]

    self.waitText = ["Warning: Center your dominant foot","to start moving the camera !"]
    self.waitTextPosition = [(100,240),(140,270)]
    self.waitTextColor = (0,128,255)

    self.currentRobot = 0
    self.useTaskAdaptation = False
    self.clutching = False
    self.wait = False


    self.imageSize = (0,0)

    self.run()


  def run(self):
    while not rospy.is_shutdown():
      if (self.useRosCvCamera and self.firstImage) or not self.useRosCvCamera:
        if self.useRosCvCamera:
          self.inputImage = self.image.copy()
        else:
          ret, self.inputImage = self.cap.read()

        self.imageSize = (self.inputImage.shape[1], self.inputImage.shape[0])

        self.outputImage = self.inputImage

        self.toolsTracker.step(self.inputImage, self.imageSize)

        result = cv2.bitwise_and(self.inputImage, self.inputImage, mask = self.toolsTracker.maskRed | self.toolsTracker.maskGreen)


        # self.toolsTracker.markersPosition[0] = np.array([630,470,1])
        self.displayMarkersPosition(self.outputImage) 
        self.displaySurgicalTaskState(self.outputImage)

        msg = Int16MultiArray()
        msg.data = np.concatenate((self.toolsTracker.markersPosition[0], self.toolsTracker.markersPosition[1]), axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPosition.publish(msg)
        
        msg = Float64MultiArray()
        msg.data = np.concatenate((self.toolsTracker.markersPositionTransformed[0], self.toolsTracker.markersPositionTransformed[1]),
                                  axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPositionTransformed.publish(msg)

        cv2.imshow('output', self.outputImage) 
        cv2.imshow('maskRed', self.toolsTracker.maskRed) 
        cv2.imshow('maskGreen', self.toolsTracker.maskGreen) 
        # cv2.imshow('result', result) 
        
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break

      self.rate.sleep()


  def displayMarkersPosition(self, image):
    for k in range(0,len(self.toolsTracker.markersPosition)):
      # if self.toolsTracker.markersPosition[k][2]:
        # cv2.circle(image, (self.markersPosition[k][0], self.markersPosition[k][1]), 5, (255, 255, 255), -1)
      cv2.drawMarker(image, (self.toolsTracker.markersPosition[k][0], self.toolsTracker.markersPosition[k][1]), (0, 255, 255),
                               cv2.MARKER_CROSS, 20, 2)
      cv2.putText(image, self.markerText[k], (self.toolsTracker.markersPosition[k][0],
                    self.toolsTracker.markersPosition[k][1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)


  def displaySurgicalTaskState(self,image):
    for k in range(0,2):
      textColor = (220,220,220)
      if k == self.currentRobot:
        textColor = self.robotColor[k] 
        cv2.rectangle(image, (0, 0), (self.imageSize[0]-1,self.imageSize[1]-1), textColor, 2)
        self.displayRobotSpecificState(image,k)

      # print(self.controlPhase[k])
      cv2.putText(image, self.robotTool[k]+self.controlPhaseText[self.controlPhase[k]], self.controlPhaseTextPosition[k], 
                  cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.9, textColor, 1)


    self.displayWarnings(image)

    
  def displayRobotSpecificState(self,image, id):
    if id == 0:
      cv2.putText(image, "Mode: "+ (self.cameraModeText[1] if self.useTaskAdaptation else self.cameraModeText[0]), 
                  self.cameraModeTextPosition, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.9, self.robotColor[id], 1)

    else:
      cv2.putText(image, "Clutching: " + (self.clutchingStateText[1] if self.clutching else self.clutchingStateText[0]), 
                  self.clutchingStateTextPosition, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.9, self.robotColor[id], 1)


  def displayWarnings(self, image):
    if self.wait:
      for k in range(0,len(self.waitTextPosition)):
        cv2.putText(image, self.waitText[k], self.waitTextPosition[k], cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.9, self.waitTextColor, 1)


  def updateSurgicalTaskState(self, msg):
    self.humanInputMode = msg.humanInputMode 
    # print(type(msg.controlPhase))
    # print(str(msg.controlPhase))
    # print(type(msg.currentRobot))
    self.controlPhase = msg.controlPhase
    self.currentRobot = msg.currentRobot
    self.useTaskAdaptation = msg.useTaskAdaptation
    self.clutching = msg.clutching
    self.wait = msg.wait


  def updateImage(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    if not self.firstImage:
      self.firstImage = True


class ToolsTracker:
  def __init__(self):
    self.lowerHsvBlue = np.array([79,71,54])     
    self.upperHsvBlue = np.array([144,255,255])

    # self.lowerHsvRed = np.array([150,60,0]) 
    # self.upperHsvRed = np.array([255,255,255]) 
    self.lowerHsvRed = np.array([0,0,40]) 
    self.upperHsvRed = np.array([8,255,255]) 

    # self.lowerHsvGreen = np.array([50,40, 0]) 
    # self.upperHsvGreen = np.array([97, 255, 255]) 
    self.lowerHsvGreen = np.array([22, 0, 40]) 
    self.upperHsvGreen = np.array([45, 255, 230]) 

    self.kernel = np.ones((5 ,5), np.uint8)

    self.markersPosition = np.array([(0,0,0),
                                    (0,0,0)])

    # self.markersPositionWindow = 
    self.markersPositionTransformed = np.array([(0.0,0.0,0),
                                               (0.0,0.0,0)])

    self.firstMarkerDetection = [False, False]



    self.alpha = 0

    self.maskRed = []
    self.maskGreen = []


  def step(self, image, imageSize):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

    # Apply red filter
    self.maskRed = self.colorFilter(hsv, imageSize, self.lowerHsvRed, self.upperHsvRed, 0)

    # Apply green filter
    self.maskGreen = self.colorFilter(hsv, imageSize, self.lowerHsvGreen, self.upperHsvGreen, 1)

    # Check for saturation
    self.checkForSaturation(image)

    # print("Red: ", image[self.markersPosition[0][1],self.markersPosition[0][0]].mean(), hsv[self.markersPosition[0][1],self.markersPosition[0][0],2])
    # print("Green: ", image[self.markersPosition[1][1],self.markersPosition[1][0]].mean(), hsv[self.markersPosition[0][1],self.markersPosition[0][0],2])


  def colorFilter(self, hsv, imageSize, lowerHsvColor, upperHsvColor, id):
    mask = cv2.inRange(hsv, lowerHsvColor, upperHsvColor) 

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    variance = cv2.Laplacian(mask, cv2.CV_64F).var()
    # print("variance: ", variance)

    _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)!=0:
      c = max(contours, key = cv2.contourArea)
      M = cv2.moments(c)

      if(M["m00"]>500 and M["m00"]<40000 and variance < 1000):
          cX = int(M["m10"] / M["m00"])
          cY = int(M["m01"] / M["m00"])

          d = np.sqrt(pow(cX-self.markersPosition[id][0],2)+pow(cY-self.markersPosition[id][1],2))
          if (d < 100 and self.markersPosition[id][2] == 1) or (self.markersPosition[id][2] == 0) or not self.firstMarkerDetection[id]:
            self.markersPosition[id][0] = int(self.alpha*float(self.markersPosition[id][0])+(1-self.alpha)*float(cX))
            self.markersPosition[id][1] = int(self.alpha*float(self.markersPosition[id][1])+(1-self.alpha)*float(cY))
            self.markersPositionTransformed[id][0] = -2.0*(float(self.markersPosition[id][1]-imageSize[1]/2.0)/imageSize[1])
            self.markersPositionTransformed[id][1] = 2.0*float(float(self.markersPosition[id][0]-imageSize[0]/2.0)/imageSize[0])
            self.markersPosition[id][2] = 1
            self.markersPositionTransformed[id][2] = 1

            if not self.firstMarkerDetection[id]:
              self.firstMarkerDetection[id] = True 
      else:
        self.markersPosition[id][2] = 0
        self.markersPositionTransformed[id][2] = 0
    else:
      self.markersPosition[id][2] = 0
      self.markersPositionTransformed[id][2] = 0

    return mask


  def checkForSaturation(self, image):
    for k in range(0,len(self.markersPosition)):
      if self.markersPosition[k][2]:
        if image[self.markersPosition[k][1],self.markersPosition[k][0]].mean() > 150:
          self.markersPosition[k][2] = 0
          self.markersPositionTransformed[k][2] = 0


if __name__ == '__main__':
  rospy.init_node('camera_manager', anonymous=True)
  cameraManager = CameraManager()
  cv2.destroyAllWindows()    