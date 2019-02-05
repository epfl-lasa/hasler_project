#!/usr/bin/env python
# license removed for brevity
import rospy
import Tkinter
import math
from std_msgs.msg import String
from std_msgs.msg import Float32

class CurrentTimeDisplay():
  # Must have __init__(self) function for a class, similar to a C++ class constructor.
  def __init__(self):
    # Create a dynamic reconfigure server.
    self.sub = rospy.Subscriber('/currentTime', Float32, self.callback)
    self.rate = 200
    self.window = Tkinter.Tk()
    self.labelText = "ElapsedTime: "
    self.label = Tkinter.Label(self.window, text=self.labelText, font=("Courier", 44))
    self.label.pack()
    self.window.mainloop()

  def callback(self,msg):
    self.label.config(text="Elapsed Time: "+str(math.floor(msg.data*10)/10))
                
if __name__ == '__main__':
  # Initialize the node and name it.
  rospy.init_node('currentTimeDisplay')
  # Go to class functions that do all the heavy lifting. Do error checking.
  try:
    ne = CurrentTimeDisplay()
  except rospy.ROSInterruptException: 
    pass