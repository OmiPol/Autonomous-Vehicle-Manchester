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
       self.hsv_img = np.uint16((self.width, self.height))
       self.edges_img = np.uint16((self.width, self.height))
       self.color = "N/A"
   
   def camera_callback(self,msg):
       self.vid = self.bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8

   def timer_callback(self):
       msg = String()
       if self.vid is not None:
           self.gray_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2GRAY)
           self.hsv_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2HSV)
           params = cv2.SimpleBlobDetector_Params()
           #area
           params.filterByArea = True;
           params.minArea = 500;
           params.maxArea = 3000;
           #inertia
           params.filterByInertia = True;
           params.minInertiaRatio = 0.005;
           detector = cv2.SimpleBlobDetector_create(params)
           keypoints = detector.detect(self.gray_img)
           if keypoints is not None:
               for kp in keypoints:
                    x = int(kp.pt[0]);
                    y = int(kp.pt[1]);
                    pixel_center = self.hsv_img[y,x]
                    hue_value = pixel_center[0]  
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

           self.im_with_keypoints = cv2.drawKeypoints(self.vid, keypoints,np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
           cv2.imshow("Semaforo", self.vid)
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
