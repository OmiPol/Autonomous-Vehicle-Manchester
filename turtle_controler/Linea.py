#!/usr/bin/env python3
import cv2, rclpy, time
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32

class LineDetect(Node):
   def __init__(self):
       super().__init__("image_processing")
       self.get_logger().info("Cam Subscriber Started...!!!")
       self.vid = None
       self.sub = self.create_subscription(Image, '/video_source/raw',self.camera_callback,10)
       self.pub = self.create_publisher(Int32, '/line_error', 10)
       self.timer = self.create_timer(0.05, self.timer_callback)
       self.width, self.height = 320,180
       self.color_img = np.uint16((self.width, self.height, 3))
       self.gray_img = np.uint16((self.width, self.height))
       self.edges_img = np.uint16((self.width, self.height))
       self.bridge = CvBridge()
       self.line = "N/A"
   
   def camera_callback(self,msg):
       self.vid = self.bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8

   def timer_callback(self):
       msg = Int32()
       if self.vid is not None:
           
           self.gray_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2GRAY)
           roi = self.gray_img[(self.height)*2//3:, :]
           blurred = cv2.GaussianBlur(roi, (5, 5), 0)
           _, mask = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV) # threshold for black color

           #centro del roi
           cx = self.width // 2
           cy = self.height // 3

           error = 69420

           # find contours of the mask
           contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
           for contour in contours:
               # get centroid of the contour
               M = cv2.moments(contour)
               if M["m00"] != 0:
                   cX = int(M["m10"] / M["m00"])
                   cY = int(M["m01"] / M["m00"])
               else:
                   cX, cY = 0, 0

               # calculate error
               temp_error = cX - cx

               # update minimum error
               if abs(temp_error) < abs(error):
                   error = temp_error

               cv2.drawContours(mask, [contour], -1, (0, 255, 0), 2)

           msg.data = int(error)
           self.pub.publish(msg)
           self.get_logger().info(f"Error mínimo: {error} píxeles")
           cv2.imshow("Puzzlebot", mask)
           cv2.waitKey(1)
           
def main(args=None):
   rclpy.init(args=args)
   nodeh = LineDetect()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error) 
   except KeyboardInterrupt: print("Node DEAD AF!!!")


if __name__ == "__main__":
   main()

