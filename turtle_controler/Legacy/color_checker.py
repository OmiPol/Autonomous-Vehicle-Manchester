#!/usr/bin/env python3
import cv2, rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OpenCVBridge(Node):

	def empty(a):
		pass

	def __init__(self):
		super().__init__("image_processing")
		self.get_logger().info("Image Subscription Node started...!!!")
		self.img = None
		self.bridge = CvBridge()
		self.sub = self.create_subscription(Image,'/video_source/raw', self.camera_callback, 10)
		self.timer = self.create_timer(0.05, self.timer_callback)
		cv2.namedWindow("Range HSV")
		cv2.resizeWindow("Range HSV", 500, 350)
		cv2.createTrackbar("HUE Min", "Range HSV", 0,180,self.empty)
		cv2.createTrackbar("HUE Max", "Range HSV", 180,180,self.empty)
		cv2.createTrackbar("SAT Min", "Range HSV", 0,255,self.empty)
		cv2.createTrackbar("SAT Max", "Range HSV", 255,255,self.empty)
		cv2.createTrackbar("VALUE Min", "Range HSV", 0,255,self.empty)
		cv2.createTrackbar("VALUE Max", "Range HSV", 255,255,self.empty)
    
	def camera_callback(self,msg):
		self.img = self.bridge.imgmsg_to_cv2(msg,"bgr8") 
  	
	def timer_callback(self):
		if self.img is not None:
			h_min = cv2.getTrackbarPos("HUE Min", "Range HSV")
			h_max = cv2.getTrackbarPos("HUE Max", "Range HSV")
			s_min = cv2.getTrackbarPos("SAT Min", "Range HSV")
			s_max = cv2.getTrackbarPos("SAT Max", "Range HSV")
			v_min = cv2.getTrackbarPos("VALUE Min", "Range HSV")
			v_max = cv2.getTrackbarPos("VALUE Max", "Range HSV")

			lower_range = np.array([h_min,s_min,v_min])
			upper_range = np.array([h_max, s_max, v_max])
			hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
			thresh = cv2.inRange(hsv, lower_range, upper_range)
			bitwise = cv2.bitwise_and(self.img,self.img, mask=thresh)
			cv2.imshow("Original Image", self.img)
			cv2.imshow("Thresholded", thresh)
			cv2.imshow("Bitwise", bitwise)
			cv2.waitKey(1)
			
def main(args=None):
	rclpy.init(args=args)
	nodeh = OpenCVBridge()
	try: rclpy.spin(nodeh)
	except Exception as error: print(error) 
	except KeyboardInterrupt: print("Node DEAD AF!!!")


if __name__ == "__main__":
	main()
  
