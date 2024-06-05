#!/usr/bin/env python3
import cv2, rclpy, time
import numpy as np
import time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String

class LineDetect(Node):
   def __init__(self):
       super().__init__("image_processing")
       self.get_logger().info("Cam Subscriber Started...!!!")
       self.vid = None
       self.sub = self.create_subscription(Image, '/video_source/raw',self.camera_callback,10)
       self.pub_linea = self.create_publisher(Int32, '/line_error', 10)
       self.pub_state = self.create_publisher(String, '/lineas', 10)
       self.timer1 = self.create_timer(0.05, self.timer_callback_zebras)
       self.timer2 = self.create_timer(0.05, self.timer_callback_line)
       self.width, self.height = 320,180
       self.color_img = np.uint16((self.width, self.height, 3))
       self.gray_img = np.uint16((self.width, self.height))
       self.edges_img = np.uint16((self.width, self.height))
       self.bridge = CvBridge()
       self.line = "N/A"
   
   def camera_callback(self,msg):
       self.vid = self.bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8

   def timer_callback_zebras(self): #Callback de deteccion de zebra
    msg = String()
    if self.vid is not None:
        self.gray_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2GRAY)
        roi = self.gray_img[(self.height)*3//4:, self.width//4:3*self.width//2] # ROI en el tercio medio de la imagen
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV) # threshold for black color

        #centro del roi
        cx = self.width // 2
        cy = self.height // 3

        # find contours of the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        centroids = []

        for contour in contours:
            # get centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centroids.append((cX, cY))

                cv2.circle(mask, (cX, cY), 5, (0, 255, 0), -1)
            
        
        # sort centroids by their y-coordinate
        centroids.sort(key=lambda x: x[1])

        # check if there are three centroids in parallel
        if len(centroids) >= 4 and len(centroids) <= 7:
            for i in range(len(centroids) - 2):
                if abs(centroids[i][1] - centroids[i+1][1]) <= 5 and abs(centroids[i][1] - centroids[i+2][1]) <= 5:
                    msg.data = "zebras"
                    self.pub_state.publish(msg)
        else:
            pass

        cv2.imshow("Cruces", mask)
        cv2.imshow("Full", self.vid)
        cv2.waitKey(1)

   def timer_callback_line(self):
    msg = Int32()
    if self.vid is not None:
        self.gray_img = cv2.cvtColor(self.vid, cv2.COLOR_BGR2GRAY)
        roi = self.gray_img[(self.height)*2//3:, :]
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV) # threshold for black color

        #centro del roi
        cx = self.width // 2
        cy = self.height // 3

        error = None

        # find contours of the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        min_distance = None
        closest_cX = None
        closest_cY = None

        for contour in contours:
            # get centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # calculate distance from the centroid to the center of ROI
            distance = ((cX - cx) ** 2 + (cY - cy) ** 2) ** 0.5

            # update minimum distance and closest centroid
            if min_distance is None or distance < min_distance:
                min_distance = distance
                closest_cX = cX
                closest_cY = cY

        if closest_cX is not None:
            # calculate error
            error = closest_cX - cx

            # draw a circle at the closest centroid
            cv2.circle(mask, (closest_cX, closest_cY), 5, (0, 255, 0), -1)

        if error is None:
            error = 69420

        msg.data = int(error)
                
        self.pub_linea.publish(msg)
        cv2.imshow("Full", self.vid)
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


