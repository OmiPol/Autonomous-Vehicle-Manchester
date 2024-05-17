#!/usr/bin/env python3
import cv2, rclpy, time
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

class OpenCVBridge(Node):
   def __init__(self):
       super().__init__("image_processing")
       self.get_logger().info("Cam Subscriber Started...!!!")
       self.vid = None
       self.bridge = CvBridge()
       self.sub = self.create_subscription(Image, '/video_source/raw',self.camera_callback,10)
       self.pub = self.create_publisher(String, '/streetlight', 10)
       self.timer = self.create_timer(0.05, self.timer_callback)
       self.width, self.height = 320,180
       self.color_img = np.uint16((self.width, self.height, 3))
       self.gray_img = np.uint16((self.width, self.height))
       self.edges_img = np.uint16((self.width, self.height))
       self.color = "N/A"
   
   def camera_callback(self,msg):
       self.vid = self.bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8

   def timer_callback(self):
       msg = String()
       if self.vid is not None:
           self.gray_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2GRAY)
           rows = self.gray_img.shape[0]
           circles = cv2.HoughCircles(self.gray_img,cv2.HOUGH_GRADIENT, dp=1, minDist=rows/8, param1=90, param2=65, minRadius=10, maxRadius=70)
           if circles is not None:
               circles = np.uint16(np.around(circles))
               radius_centre = 1
               thickness_centre = 2
               thickness_outline = 2
               for i in circles[0, :]:
                   hsv_frame = cv2.cvtColor(self.vid, cv2.COLOR_BGR2HSV)
                   cx = int(self.width / 2)
                   cy = int(self.height / 2)
                   pixel_center = hsv_frame[cy,cx]
                   hue_value = pixel_center[0]

                   center = (i[0], i[1])
                   cv2.circle(self.vid, center, radius_centre,(255,255,255),thickness_centre)
                   radius_outline = i[2]
                   cv2.circle(self.vid, center, radius_outline, (255,255,255),thickness_outline)
                   if (hue_value >= 0 and hue_value < 12) or (hue_value >= 165 and hue_value < 181):
                       self.color = "RED"
                       msg.data = 'r'
                       self.pub.publish(msg)

                   elif (hue_value >= 12 and hue_value < 30):
                       self.color = "YELLOW"
                       msg.data = 'y'
                       self.pub.publish(msg)
                   
                   elif (hue_value >= 75 and hue_value < 100):
                       self.color = "GREEN"
                       msg.data = 'g'
                       self.pub.publish(msg)

                   elif ((hue_value >= 0 and hue_value < 12) or (hue_value >= 165 and hue_value < 181))and (hue_value >= 75 and hue_value < 100):
                       self.color = "RED"
                       msg.data = 'r'
                       self.pub.publish(msg)
                    
                   elif ((hue_value >= 0 and hue_value < 12) or (hue_value >= 165 and hue_value < 181))and (hue_value >= 12 and hue_value < 30):
                       self.color = "RED"
                       msg.data = 'r'
                       self.pub.publish(msg)


           else:
               self.color = "No Light Detected!!!"

           cv2.imshow("Puzzlebot", self.vid)
           #self.get_logger().info(self.color)
           cv2.waitKey(1)
           
def main(args=None):
   rclpy.init(args=args)
   nodeh = OpenCVBridge()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error) 
   except KeyboardInterrupt: print("Node killed unu")


if __name__ == "__main__":
   main()
