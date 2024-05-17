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
           edges = cv2.Canny(blurred,50,150,apertureSize=3)
           lines = cv2.HoughLinesP(edges,1,np.pi/180,50,minLineLength=40,maxLineGap=10)
           
           #centro del roi
           cx = self.width // 2
           cy = self.height // 3

           error = 69420

           if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    
                    #centro de linea
                    mx = (x1 + x2) // 2
                    my = (y1 + y2) // 2

                    #calculo de error
                    temp_error = mx - cx

                    #actualizacion del minimo    
                    if abs(temp_error) < abs(error):
                        error = temp_error

                    cv2.line(self.gray_img, (x1, y1 + (self.height*2)//3), (x2, y2 + (self.height*2)//3), (0, 255, 0), 2)

                    msg.data = int(error)
                    
           self.pub.publish(msg)
           self.get_logger().info(f"Error mínimo: {error} píxeles")
           cv2.imshow("Puzzlebot", self.gray_img)
           cv2.waitKey(1)
           
def main(args=None):
   rclpy.init(args=args)
   nodeh = LineDetect()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error) 
   except KeyboardInterrupt: print("Node DEAD AF!!!")


if __name__ == "__main__":
   main()