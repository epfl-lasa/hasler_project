import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from surgical_task.msg import SurgicalTaskStateMsg


class ToolsTracker:
  def __init__(self):

    self.image = []

    self.lowerHsvBlue = np.array([79,71,54])     
    self.upperHsvBlue = np.array([144,255,255])

    self.lowerHsvRed = np.array([150,60,0]) 
    self.upperHsvRed = np.array([255,255,255]) 

    self.lowerHsvGreen = np.array([50,40, 0]) 
    self.upperHsvGreen = np.array([97, 255, 255]) 

    self.kernel = np.ones((5 ,5), np.uint8)

    self.rate = rospy.Rate(30) 

    self.firstImage = True

    self.imageSize = []

    self.alpha = 0

    self.bridge = CvBridge()

    self.pubMarkersPositionTransformed = rospy.Publisher('surgical_task/markers_position_transformed', Float64MultiArray, queue_size=1)
    self.pubMarkersPosition = rospy.Publisher('endoscope_modifier/markers_position', Int16MultiArray, queue_size=1)

    # self.subCamera = rospy.Subscriber('/cv_camera/image_raw', Image, self.updateImage)
    self.cap = cv2.VideoCapture("/dev/video0")


    self.output = []

    self.markersPosition = np.array([[0,0,0],
                                    [0,0,0]])

    self.markersPositionTransformed = np.array([[0.0,0.0,0],
                                               [0.0,0.0,0]])
    self.markerText = ["R","G"]

    cv2.namedWindow("output", cv2.WINDOW_NORMAL)  
    self.run()



  def run(self):
    while not rospy.is_shutdown():
      if self.firstImage:

        ret, output = self.cap.read()
        self.imageSize = output.shape

        # output = self.image.copy()

        hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV) 

        # Apply red filter
        maskRed = self.colorFilter(hsv, self.lowerHsvRed, self.upperHsvRed, 0)

        # Apply green filter
        maskGreen = self.colorFilter(hsv, self.lowerHsvGreen, self.upperHsvGreen, 1)

        result = cv2.bitwise_and(output, output, mask = maskRed | maskGreen)

        self.showMarkersPosition(output) 


        msg = Int16MultiArray()
        msg.data = np.concatenate((self.markersPosition[0], self.markersPosition[1]), axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPosition.publish(msg)
        
        msg = Float64MultiArray()
        msg.data = np.concatenate((self.markersPositionTransformed[0], self.markersPositionTransformed[1]), axis=None)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].size = len(msg.data)
        self.pubMarkersPositionTransformed.publish(msg)

        cv2.imshow('output', output) 
        # cv2.imshow('maskRed', maskRed) 
        # cv2.imshow('maskGreen', maskGreen) 
        # cv2.imshow('result', result) 
        
      self.rate.sleep()

      if cv2.waitKey(1) & 0xFF == ord('q'):
          break


  def colorFilter(self, hsv, lowerHsvColor, upperHsvColor, id):
    mask = cv2.inRange(hsv, lowerHsvColor, upperHsvColor) 

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    variance = cv2.Laplacian(mask, cv2.CV_64F).var()
    # print("variance: ", variance)

    _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    self.markersPosition[id][2] = 0
    self.markersPositionTransformed[id][2] = 0

    if len(contours)!=0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)

        if(M["m00"]>500 and M["m00"]<40000 and variance < 1000):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            self.markersPosition[id][0] = int(self.alpha*float(self.markersPosition[id][0])+(1-self.alpha)*float(cX))
            self.markersPosition[id][1] = int(self.alpha*float(self.markersPosition[id][1])+(1-self.alpha)*float(cY))
            self.markersPosition[id][2] = 1
            self.markersPositionTransformed[id][0] = -2.0*(float(self.markersPosition[id][1]-self.imageSize[0]/2.0)/self.imageSize[0])
            self.markersPositionTransformed[id][1] = 2.0*float(float(self.markersPosition[id][0]-self.imageSize[1]/2.0)/self.imageSize[1])
            self.markersPositionTransformed[id][2] = 1

    return mask


  def showMarkersPosition(self, image):
    for k in range(0,len(self.markersPosition)):
      if self.markersPosition[k][2]:
        # cv2.circle(image, (self.markersPosition[k][0], self.markersPosition[k][1]), 5, (255, 255, 255), -1)
        cv2.drawMarker(image, (self.markersPosition[k][0], self.markersPosition[k][1]), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(image, self.markerText[k], (self.markersPosition[k][0], self.markersPosition[k][1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


  def updateImage(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    if not self.firstImage:
      self.firstImage = True
      self.imageSize = self.image.shape



if __name__ == '__main__':
  rospy.init_node('tools_tracker', anonymous=True)
  toolsTracker = ToolsTracker()
  cv2.destroyAllWindows()    