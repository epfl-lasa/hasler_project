#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from surgical_task.msg import SurgicalTaskStateMsg, RobotStateMsg
import time
import rospkg

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

    self.cameraWorkspaceCollisionText = "Workspace limits !"
    self.cameraWorkspaceCollisionTextPosition = (65,90) 
    self.showCameraWorkspaceCollisionText = False
    self.timeCameraWorkspaceCollisionText = time.time()

    self.clutchingStateText = ["Off", "On"]
    self.clutchingStateTextPosition = (440,60)

    self.markerText = ["R", "G", "Y"]
    self.markerColor = (0, 255, 255)


    self.waitText = ["Center your dominant foot to" ,"start moving the camera !"]
    self.waitTextPosition = [(160,240),(180,270)]
    self.waitTextColor = (0,100,255)


    # self.eeCollisionText = ["Warning: The robots' end-effectors", "are close to collide !"]
    # self.eeCollisionTextPosition = [(130,270),(210,300)]
    self.eeCollisionText = ["End-effectors collision !"]
    self.eeCollisionTextPosition = [(45,420)]
    self.eeCollisionTextColor = (0,100,255)
    self.showEECollisionText = False
    self.timeEECollisionText = time.time()


    # self.toolCollisionText = ["Warning: The camera and instrument", "are close to collide !"]
    # self.toolCollisionTextPosition = [(130,330),(210,360)]
    self.toolCollisionText = ["Camera-Instrument collision !"]
    self.toolCollisionTextPosition = [(45,460)]
    self.toolCollisionTextColor = (0,100,255)
    self.showToolCollisionText = False
    self.timeToolCollisionText = time.time()

    rospack = rospkg.RosPack()
    rospack.list() 

    self.warningImage = cv2.imread(rospack.get_path('endoscope_feedback')+"/src/img/warning.png", cv2.IMREAD_UNCHANGED)
    self.warningImage = cv2.resize(self.warningImage, (40,40), interpolation = cv2.INTER_AREA)

    self.currentRobot = 0
    self.useTaskAdaptation = False
    self.clutching = False
    self.wait = False
    self.eeCollision = [False, False]
    self.toolCollision = [False, False]
    self.workspaceCollision = [False, False]
    self.beliefsC = [1,0,0]


    self.imageSize = (0,0)

    self.pubMarkersPositionTransformed = rospy.Publisher('surgical_task/markers_position_transformed', Float64MultiArray, 
                                                         queue_size=1)
    self.pubMarkersPosition = rospy.Publisher('endoscope_modifier/markers_position', Int16MultiArray, queue_size=1)
    self.subSurgicalTaskState = rospy.Subscriber('surgical_task/state', SurgicalTaskStateMsg, self.updateSurgicalTaskState)

    self.subRobotState = []
    self.subRobotState.append(rospy.Subscriber('/surgical_task/left_robot_state', RobotStateMsg, self.updateRobotState, 0))
    self.subRobotState.append(rospy.Subscriber('/surgical_task/right_robot_state', RobotStateMsg, self.updateRobotState, 1))

    self.run()


  def run(self):
    while not rospy.is_shutdown():
      if (self.useRosCvCamera and self.firstImage) or not self.useRosCvCamera:
        t0 = time.time()
        if self.useRosCvCamera:
          self.inputImage = self.image.copy()
        else:
          ret, self.inputImage = self.cap.read()

        # print("Read camera:", time.time()-t0)
        self.imageSize = (self.inputImage.shape[1], self.inputImage.shape[0])



        self.outputImage = self.inputImage

        t1 = time.time()
        self.toolsTracker.step(self.inputImage, self.imageSize)

        # print("Track tool:", time.time()-t1)



        # result = cv2.bitwise_and(self.inputImage, self.inputImage, mask = self.toolsTracker.maskRed | self.toolsTracker.maskGreen 
        #                                                                   | self.toolsTracker.maskYellow)



        t2 = time.time()
        # self.toolsTracker.tipPosition[0] = np.array([630,470,1])
        self.displayMarkersPosition(self.outputImage, False) 
        self.displaySurgicalTaskState(self.outputImage)
        # print("Fill images:", time.time()-t2)

        t3 = time.time()

        msg = Int16MultiArray()
        msg.data = np.concatenate((self.toolsTracker.tipPosition[0], self.toolsTracker.tipPosition[1], 
                                   self.toolsTracker.tipPosition[2]), axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPosition.publish(msg)
        
        msg = Float64MultiArray()
        msg.data = np.concatenate((self.toolsTracker.tipPositionTransformed[0], self.toolsTracker.tipPositionTransformed[1],
                                   self.toolsTracker.tipPositionTransformed[2]), axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPositionTransformed.publish(msg)

        # print("Pub data:", time.time()-t3)

        t4 = time.time()

        cv2.imshow('output', self.outputImage) 
        cv2.imshow('maskRed', self.toolsTracker.maskRed) 
        cv2.imshow('maskBlue', self.toolsTracker.maskBlue) 
        cv2.imshow('maskGreen', self.toolsTracker.maskGreen) 
        cv2.imshow('maskOrange', self.toolsTracker.maskOrange) 
        cv2.imshow('maskYellow', self.toolsTracker.maskYellow) 
        cv2.imshow('maskCyan', self.toolsTracker.maskCyan) 
        # cv2.imshow('result', result) 
        
        # print("Show images:", time.time()-t4)

        if(time.time()-t0>0.1):
          print("Warning: camera delay !!!")

      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

      self.rate.sleep()


  # def displayMarkersPosition(self, image):
  #   for k in range(0,len(self.toolsTracker.tipPosition)):
  #     color = (255, 255, 255)
  #     if self.toolsTracker.tipPosition[k][2]:
  #       color = self.markerColor
  #       if self.useTaskAdaptation and (1.0-self.beliefsC[k])<1e-3:
  #         color = (255, 0, 255)


  #     cv2.drawMarker(image, (self.toolsTracker.tipPosition[k][0], self.toolsTracker.tipPosition[k][1]), color,
  #                              cv2.MARKER_CROSS, 20, 2)

  #     pY = self.toolsTracker.tipPosition[k][1] - 25
  #     if pY < 10:
  #       pY = self.toolsTracker.tipPosition[k][1] + 25

  #     cv2.putText(image, self.markerText[k], (self.toolsTracker.tipPosition[k][0], pY), cv2.FONT_HERSHEY_TRIPLEX, 0.6, color, 2)


  def displayMarkersPosition(self, image, showTip=False):
    for k in range(0,len(self.toolsTracker.tipPosition)):
      color = (255, 255, 255)
      if self.toolsTracker.tipPosition[k][2]:
        color = self.markerColor
        if self.useTaskAdaptation:
          trackedColor = (255,0,255)
          color = (1-self.beliefsC[k])*np.array(color).astype(np.float32)+self.beliefsC[k]*np.array(trackedColor).astype(np.float32)
          color = color.astype(int).tolist()

      position = self.toolsTracker.markerPosition[k][0:2]
      if showTip:
        position = self.toolsTracker.tipPosition[k][0:2]

      cv2.drawMarker(image, (int(position[0]), int(position[1])), color,cv2.MARKER_CROSS, 20, 2)

      pY = int(position[1]) - 25
      if pY < 10:
        pY = int(position[1]) + 25

      cv2.putText(image, self.markerText[k], (int(position[0]), pY), cv2.FONT_HERSHEY_TRIPLEX, 0.6, color, 2)

  def displaySurgicalTaskState(self,image):
    for k in range(0,2):
      textColor = (220,220,220)
      if k == self.currentRobot and self.humanInputMode == 1:
        textColor = self.robotColor[k] 
        cv2.rectangle(image, (0, 0), (self.imageSize[0]-1,self.imageSize[1]-1), textColor, 2)
        self.displayRobotSpecificState(image,k)
      elif self.humanInputMode == 0:
        textColor = self.robotColor[k]
        self.displayRobotSpecificState(image,k)

      # print(self.controlPhase[k])
      cv2.putText(image, self.robotTool[k]+self.controlPhaseText[self.controlPhase[k]], self.controlPhaseTextPosition[k], 
                  cv2.FONT_HERSHEY_TRIPLEX, 0.6, textColor, 1)

    self.displayWarnings(image)

    
  def displayRobotSpecificState(self,image, id):
    if id == 0:
      cv2.putText(image, "Mode: "+ (self.cameraModeText[1] if self.useTaskAdaptation else self.cameraModeText[0]), 
                  self.cameraModeTextPosition, cv2.FONT_HERSHEY_TRIPLEX, 0.6, self.robotColor[id], 1)

    else:
      cv2.putText(image, "Clutching: " + (self.clutchingStateText[1] if self.clutching else self.clutchingStateText[0]), 
                  self.clutchingStateTextPosition, cv2.FONT_HERSHEY_TRIPLEX, 0.6, self.robotColor[id], 1)


  def displayWarnings(self, image):
    if self.wait:
      self.overlay_image_alpha(image, self.warningImage, (self.waitTextPosition[0][0]-45,self.waitTextPosition[0][1]-25), self.warningImage[:,:,3]/255)

      for k in range(0,len(self.waitTextPosition)):
        cv2.putText(image, self.waitText[k], self.waitTextPosition[k], cv2.FONT_HERSHEY_TRIPLEX, 0.6, self.waitTextColor, 1)

    if self.eeCollision[0]:
      if time.time()-self.timeEECollisionText> 1:
        self.showEECollisionText = not self.showEECollisionText
        self.timeEECollisionText = time.time()
      if self.showEECollisionText:
        self.overlay_image_alpha(image, self.warningImage, (self.eeCollisionTextPosition[0][0]-45,self.eeCollisionTextPosition[0][1]-25), self.warningImage[:,:,3]/255)
        for k in range(0,len(self.eeCollisionTextPosition)):
          cv2.putText(image, self.eeCollisionText[k], self.eeCollisionTextPosition[k], cv2.FONT_HERSHEY_TRIPLEX, 0.6, self.eeCollisionTextColor, 1)

    if self.toolCollision[0]:
      if time.time()-self.timeToolCollisionText> 1:
        self.showToolCollisionText = not self.showToolCollisionText
        self.timeToolCollisionText = time.time()
      if self.showToolCollisionText:
        self.overlay_image_alpha(image, self.warningImage, (self.toolCollisionTextPosition[0][0]-45,self.toolCollisionTextPosition[0][1]-25), self.warningImage[:,:,3]/255)
        for k in range(0,len(self.toolCollisionTextPosition)):
          cv2.putText(image, self.toolCollisionText[k], self.toolCollisionTextPosition[k], cv2.FONT_HERSHEY_TRIPLEX, 0.6, self.toolCollisionTextColor, 1)

    if self.workspaceCollision[0]:
      if time.time()-self.timeCameraWorkspaceCollisionText> 1:
        self.showCameraWorkspaceCollisionText = not self.showCameraWorkspaceCollisionText
        self.timeCameraWorkspaceCollisionText = time.time()
      if self.showCameraWorkspaceCollisionText:
        textColor = self.robotColor[0] 
        self.overlay_image_alpha(image, self.warningImage, (self.cameraWorkspaceCollisionTextPosition[0]-45,self.cameraWorkspaceCollisionTextPosition[1]-25), self.warningImage[:,:,3]/255)
        cv2.putText(image, self.cameraWorkspaceCollisionText, self.cameraWorkspaceCollisionTextPosition, 
                    cv2.FONT_HERSHEY_TRIPLEX, 0.6, textColor, 1)

  def updateSurgicalTaskState(self, msg):
    self.humanInputMode = msg.humanInputMode 
    self.currentRobot = msg.currentRobot
    self.useTaskAdaptation = msg.useTaskAdaptation
    self.clutching = msg.clutching
    self.wait = msg.wait
    self.beliefsC = msg.beliefsC


  def updateRobotState(self,msg, r):
    self.controlPhase[r] = msg.controlPhase
    self.eeCollision[r] = msg.eeCollisionConstraintActive
    self.toolCollision[r] = msg.toolCollisionConstraintActive
    self.workspaceCollision[r] = msg.workspaceCollisionConstraintActive


  def updateImage(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    if not self.firstImage:
      self.firstImage = True



  def overlay_image_alpha(self, img, img_overlay, pos, alpha_mask):
      """Overlay img_overlay on top of img at the position specified by
      pos and blend using alpha_mask.

      Alpha mask must contain values within the range [0, 1] and be the
      same size as img_overlay.
      """

      x, y = pos

      # Image ranges
      y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
      x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])

      # Overlay ranges
      y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
      x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)

      # Exit if nothing to do
      if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
          return

      channels = img.shape[2]

      alpha = alpha_mask[y1o:y2o, x1o:x2o]
      alpha_inv = 1.0 - alpha

      for c in range(channels):
          img[y1:y2, x1:x2, c] = (alpha * img_overlay[y1o:y2o, x1o:x2o, c] +
                                  alpha_inv * img[y1:y2, x1:x2, c])


class ToolsTracker:
  def __init__(self):
    self.lowerHsvBlue = np.array([97,20,20])     
    self.upperHsvBlue = np.array([120,255,255])

    self.lowerHsvCyan = np.array([40,0,0])     
    self.upperHsvCyan = np.array([97,255,255])

    # self.lowerHsvRed = np.array([150,60,0]) 
    # self.upperHsvRed = np.array([255,255,255]) 
    # self.lowerHsvRed = np.array([0,10,100]) 
    # self.upperHsvRed = np.array([10,255,255]) 
    # self.lowerHsvRed = np.array([112,0,0]) 
    # self.upperHsvRed = np.array([179,110,255]) 
    self.lowerHsvRed = np.array([0,0,20]) 
    self.upperHsvRed = np.array([8,255,255]) 
    # self.lowerHsvRed = np.array([0,120,0]) 
    # self.upperHsvRed = np.array([179,255,255]) 

    # self.lowerHsvGreen = np.array([50,40, 0]) 
    # self.upperHsvGreen = np.array([97, 255, 255]) 
    # self.lowerHsvGreen = np.array([44, 0, 0]) 
    # self.upperHsvGreen = np.array([69, 255, 100]) 

    # self.lowerHsvGreen = np.array([44, 0, 0]) 
    # self.upperHsvGreen = np.array([69, 255, 100]) 
    # self.lowerHsvGreen = np.array([80, 0, 80]) 
    # self.upperHsvGreen = np.array([100, 255, 170]) 
    # self.lowerHsvGreen = np.array([80, 60, 80]) 
    # self.upperHsvGreen = np.array([105, 255, 140]) 
    # self.lowerHsvGreen = np.array([25, 117, 30]) 
    # self.upperHsvGreen = np.array([44, 255, 255]) 
    self.lowerHsvGreen = np.array([20, 120, 23]) 
    self.upperHsvGreen = np.array([80, 255, 170]) 
    # self.lowerHsvGreen = np.array([31, 0, 0]) 
    # self.upperHsvGreen = np.array([70, 255, 255]) 


    # self.lowerHsvRed = np.array([0,0,40]) 
    # self.upperHsvRed = np.array([8,255,255]) 
    # self.lowerHsvGreen = np.array([22, 0, 40]) 
    # self.upperHsvGreen = np.array([45, 255, 230]) 

    # self.lowerHsvYellow = np.array([0,0,0])
    # self.upperHsvYellow = np.array([80,255,255])
    # self.lowerHsvYellow = np.array([17,130,170])
    # self.upperHsvYellow = np.array([25,255,255])
    # self.lowerHsvYellow = np.array([13,170,120])
    # self.upperHsvYellow = np.array([25,255,255])
    self.lowerHsvYellow = np.array([15,160,120])
    self.upperHsvYellow = np.array([24,255,255])
    # self.lowerHsvYellow = np.array([10,120,0])
    # self.upperHsvYellow = np.array([179,255,255])

    self.lowerHsvOrange = np.array([8,190,100])
    self.upperHsvOrange = np.array([15,255,255])

    self.lowerHsvPink = np.array([0,0,0])
    self.upperHsvPink = np.array([8,217,255])

    self.kernel = np.ones((5 ,5), np.uint8)

    self.markerPosition = np.array([(0, 0, 0),
                                    (0, 0, 0),
                                    (0, 0, 0)])

    self.markerPositionTransformed = np.array([(0.0, 0.0, 0),
                                               (0.0, 0.0, 0),
                                               (0.0, 0.0, 0)])

    self.tipPosition = np.array([(0, 0, 0),
                                 (0, 0, 0),
                                 (0, 0, 0)])

    self.dir = np.array([(0.0, 0.0),
                         (0.0, 0.0),
                         (0.0, 0.0)])

    self.tipPositionTransformed = np.array([(0.0, 0.0, 0),
                                            (0.0, 0.0, 0),
                                            (0.0, 0.0, 0)])

    self.timeMarkerDisappear = [0.0, 0.0, 0.0]

    self.firstMarkerDetection = [False, False, False]

    self.markerLength = [0.0, 0.0, 0.0]

    self.alpha = 0
    self.scale = 1.5

    # self.maskRed = []
    # self.maskGreen = []


  def step(self, image, imageSize):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

    # Apply red filter
    self.maskRed, self.maskBlue = self.colorFilterTip(hsv, imageSize, self.lowerHsvRed, self.upperHsvRed, self.lowerHsvBlue, self.upperHsvBlue, 0)
    # self.maskRed = self.colorFilter(hsv, imageSize, self.lowerHsvRed, self.upperHsvRed, 0)

    # Apply green filter
    self.maskGreen, self.maskOrange = self.colorFilterTip(hsv, imageSize, self.lowerHsvGreen, self.upperHsvGreen, self.lowerHsvOrange, self.upperHsvOrange, 1)
    # self.maskGreen = self.colorFilter(hsv, imageSize, self.lowerHsvGreen, self.upperHsvGreen, 1)


    # Apply yellow filter
    self.maskYellow, self.maskCyan = self.colorFilterTip(hsv, imageSize, self.lowerHsvYellow, self.upperHsvYellow, self.lowerHsvCyan, self.upperHsvCyan, 2)
    # self.maskYellow = self.colorFilter(hsv, imageSize, self.lowerHsvYellow, self.upperHsvYellow, 2)


    # Check for saturation
    self.checkForSaturation(image)

    # print("Red: ", image[self.tipPosition[0][1],self.tipPosition[0][0]].mean(), hsv[self.tipPosition[0][1],self.tipPosition[0][0],2])
    # print("Green: ", image[self.tipPosition[1][1],self.tipPosition[1][0]].mean(), hsv[self.tipPosition[0][1],self.tipPosition[0][0],2])


  def colorFilter(self, hsv, imageSize, lowerHsvColor, upperHsvColor, id):
    mask = cv2.inRange(hsv, lowerHsvColor, upperHsvColor) 

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    variance = cv2.Laplacian(mask, cv2.CV_64F).var()
    # print("variance: ", variance)

    # check OpenCV version
    major = cv2.__version__.split('.')[0]
    if major == '3':
      ret, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
      contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)!=0:
      c = max(contours, key = cv2.contourArea)
      M = cv2.moments(c)

      # if(M["m00"]>500 and M["m00"]<40000 and variance < 1000):
      if(M["m00"]>500 and M["m00"]<100000):
        # (x,y),(width,height),angle = cv2.fitEllipse(c)
        # print(id, angle, width,height)
        # # center,size,angle = cv2.minAreaRect(c)
        # print(angle)
        # print(id, x,y, angle, MA, ma)
        # print(id, angle, size)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # print(cX,cY)
        # cX = cX+np.sin(angle*np.pi/180)*height/2
        # cY = cY-np.cos(angle*np.pi/180)*height/2
        # cX = cX+np.cos(angle*np.pi/180)*height/2
        # cY = cY+np.sin(angle*np.pi/180)*height/2


        d = np.sqrt(pow(cX-self.tipPosition[id][0],2)+pow(cY-self.tipPosition[id][1],2))
        if (d < 200 and self.tipPosition[id][2] == 1) or (self.tipPosition[id][2] == 0) or not self.firstMarkerDetection[id]:
          self.tipPosition[id][0] = int(self.alpha*float(self.tipPosition[id][0])+(1-self.alpha)*float(cX))
          self.tipPosition[id][1] = int(self.alpha*float(self.tipPosition[id][1])+(1-self.alpha)*float(cY))
          self.tipPositionTransformed[id][0] = -2.0*(float(self.tipPosition[id][1]-imageSize[1]/2.0)/imageSize[1])
          self.tipPositionTransformed[id][1] = 2.0*float(float(self.tipPosition[id][0]-imageSize[0]/2.0)/imageSize[0])
          self.tipPosition[id][2] = 1
          self.tipPositionTransformed[id][2] = 1

          if not self.firstMarkerDetection[id]:
            self.firstMarkerDetection[id] = True 
      else:
        self.tipPosition[id][2] = 0
        self.tipPositionTransformed[id][2] = 0

    else:
      self.tipPosition[id][2] = 0
      self.tipPositionTransformed[id][2] = 0

    return mask


  def colorFilterTip(self, hsv, imageSize, lowerHsvColorTip, upperHsvColorTip, lowerHsvColorBase, upperHsvColorBase, id):

    maskBase = cv2.inRange(hsv, lowerHsvColorBase, upperHsvColorBase) 

    maskBase = cv2.morphologyEx(maskBase, cv2.MORPH_OPEN, self.kernel)

    # maskTip = cv2.morphologyEx(maskTip, cv2.MORPH_CLOSE, self.kernel)

    variance = cv2.Laplacian(maskBase, cv2.CV_64F).var()
    # print("variance: ", variance)

    # check OpenCV version
    major = cv2.__version__.split('.')[0]
    if major == '3':
      ret, contours, hierarchy = cv2.findContours(maskBase, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
      contours, hierarchy = cv2.findContours(maskBase, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # _, contours, _ = cv2.findContours(maskTip,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    xBase = np.array((0,0))
    baseMarkerDetected = False
    if len(contours)!=0:
      c = max(contours, key = cv2.contourArea)
      M = cv2.moments(c)

      # if(M["m00"]>500 and M["m00"]<40000 and variance < 1000):
      if(M["m00"]>500 and M["m00"]<100000):
        xBase[0] = int(M["m10"] / M["m00"])
        xBase[1] = int(M["m01"] / M["m00"])
        baseMarkerDetected = True

    maskTip = cv2.inRange(hsv, lowerHsvColorTip, upperHsvColorTip) 

    maskTip = cv2.morphologyEx(maskTip, cv2.MORPH_OPEN, self.kernel)

    # maskTip = cv2.morphologyEx(maskTip, cv2.MORPH_CLOSE, self.kernel)

    variance = cv2.Laplacian(maskTip, cv2.CV_64F).var()
    # print("variance: ", variance)

    # check OpenCV version
    major = cv2.__version__.split('.')[0]
    if major == '3':
      ret, contours, hierarchy = cv2.findContours(maskTip, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
      contours, hierarchy = cv2.findContours(maskTip, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # _, contours, _ = cv2.findContours(maskTip,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)!=0:
      c = max(contours, key = cv2.contourArea)
      M = cv2.moments(c)

      # if(M["m00"]>500 and M["m00"]<40000 and variance < 1000):
      # print(M["m00"])
      if(M["m00"]>470 and M["m00"]<100000):
        # self.markerLength[id] = 0*self.markerLength[id]+1*size
        self.markerLength[id] = np.sqrt(M["m00"]/(13*6))*13
        # print(id, angle, width,height)
        # center,(_,self.markerLength[id]),angle = cv2.minAreaRect(c)
        # print(angle)
        # print(id, x,y, angle, MA, ma)
        # print(id, angle, size)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # cX = cX+np.sin(angle*np.pi/180)*height/2
        # cY = cY-np.cos(angle*np.pi/180)*height/2
        # cX = cX+np.cos(angle*np.pi/180)*height/2
        # cY = cY+np.sin(angle*np.pi/180)*height/2


        # d = np.sqrt(pow(cX-self.tipPosition[id][0],2)+pow(cY-self.tipPosition[id][1],2))
        # if (d < 300 and self.tipPosition[id][2] == 1) or (self.tipPosition[id][2] == 0) or not self.firstMarkerDetection[id]:
        self.markerPosition[id][0] = int(self.alpha*float(self.markerPosition[id][0])+(1.0-self.alpha)*float(cX))
        self.markerPosition[id][1] = int(self.alpha*float(self.markerPosition[id][1])+(1.0-self.alpha)*float(cY))
        self.markerPositionTransformed[id][0] = -2.0*(float(self.markerPosition[id][1]-imageSize[1]/2.0)/imageSize[1])
        self.markerPositionTransformed[id][1] = 2.0*float(float(self.markerPosition[id][0]-imageSize[0]/2.0)/imageSize[0])
        self.markerPosition[id][2] = 1
        self.markerPositionTransformed[id][2] = 1
        if baseMarkerDetected:
          self.dir[id] = (self.markerPosition[id][0:2]-xBase)/np.linalg.norm(self.markerPosition[id][0:2]-xBase) 
          self.dir[id] = self.dir[id]/np.linalg.norm(self.dir[id])
          # print(self.dir[id].T, self.markerLength[id], np.sqrt(M["m00"]/(13*6))*13)
          # print(self.markerLength[id])
          # temp = (cX,cY)
          # print(temp-xBase, np.linalg.norm(temp-xBase))
          # print(baseMarkerDetected, self.dir[id])
        self.tipPosition[id] = self.markerPosition[id]
        self.tipPosition[id][0] = self.tipPosition[id][0]+int(self.dir[id][0]*self.scale*self.markerLength[id])
        self.tipPosition[id][1] = self.tipPosition[id][1]+int(self.dir[id][1]*self.scale*self.markerLength[id])
        self.tipPosition[id][0] = np.clip(self.tipPosition[id][0],0,imageSize[0]-1)
        self.tipPosition[id][1] = np.clip(self.tipPosition[id][1],0,imageSize[1]-1)
        self.tipPositionTransformed[id][0] = -2.0*(float(self.tipPosition[id][1]-imageSize[1]/2.0)/imageSize[1])
        self.tipPositionTransformed[id][1] = 2.0*float(float(self.tipPosition[id][0]-imageSize[0]/2.0)/imageSize[0])
        self.tipPosition[id][2] = 1
        self.tipPositionTransformed[id][2] = 1
        self.timeMarkerDisappear[id] = -1

        if not self.firstMarkerDetection[id]:
          self.firstMarkerDetection[id] = True 
      else:
        # if self.tipPosition[id][2] == 1 and self.timeMarkerDisappear[id] < 0:
        #   self.timeMarkerDisappear[id] = time.time()
        # if time.time()-self.timeMarkerDisappear[id]>1:
        self.tipPosition[id][2] = 0
        self.tipPositionTransformed[id][2] = 0
        self.markerPosition[id][2] = 0
        self.markerPositionTransformed[id][2] = 0


    else:
      # if self.tipPosition[id][2] == 1 and self.timeMarkerDisappear[id] < 0:
      #   self.timeMarkerDisappear[id] = time.time()
      # if time.time()-self.timeMarkerDisappear[id]>0.5:
      self.tipPosition[id][2] = 0
      self.tipPositionTransformed[id][2] = 0
      self.markerPosition[id][2] = 0
      self.markerPositionTransformed[id][2] = 0


    return maskTip, maskBase


  def checkForSaturation(self, image):
    for k in range(0,len(self.tipPosition)):
      if self.tipPosition[k][2]:
        temp = self.tipPosition[k][0:2]-self.dir[k]*self.scale*self.markerLength[k]
        temp[0] = np.clip(temp[0],0,image.shape[1]-1)
        temp[1] = np.clip(temp[1],0,image.shape[0]-1)

        intensity = image[int(temp[1]),int(temp[0])].mean()
        if intensity > 220:
          print("Saturation: ", k, intensity)
          self.tipPosition[k][2] = 0
          self.tipPositionTransformed[k][2] = 0


if __name__ == '__main__':
  rospy.init_node('camera_manager', anonymous=True)
  cameraManager = CameraManager()
  cv2.destroyAllWindows()    