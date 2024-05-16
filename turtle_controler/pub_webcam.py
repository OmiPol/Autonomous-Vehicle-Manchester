#! /usr/bin/env python3
import rclpy,cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class MyClassNode(Node):
   def __init__(self):
      super().__init__("webcam_publisher") #declara el nodo
      self.get_logger().info("webcam publisher started")
      self.width,self.height = 360,240
      self.vid = cv2.VideoCapture(0)
      if not self.vid.isOpened():
         self.get_logger().error("No webcam detected")
         quit()
         
      self.vid.set(cv2.CAP_PROP_FRAME_WIDTH,self.width)
      self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
      self.color_img = np.ndarray((self.width,self.height,3))
      self.gray_img = np.ndarray((self.width,self.height))
      self.bridge = CvBridge()
      self.pub = self.create_publisher(Image,"/stream",10)
      self.timer = self.create_timer(0.05,self.timer_callback)
      
      
      
   def timer_callback(self):
      ret,self.color_img = self.vid.read()
      if ret:
         self.gray_img = cv2.cvtColor(self.color_img,cv2.COLOR_BGR2GRAY)
         self.pub.publish(self.bridge.cv2_to_imgmsg(self.gray_img,encoding="mono8"))
         #self.pub.publish(self.bridge.cv2_to_imgmsg(self.color_img,encoding="bgr8"))
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = MyClassNode()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()

