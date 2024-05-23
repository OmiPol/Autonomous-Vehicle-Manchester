#! /usr/bin/env python3
import rclpy,cv2
from rclpy.node import node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OpenCVBridge(Node):
   def __init__(self):
      
      super().__init__("CV_bridge") #declara el nodo
      self.get_logger().info("Node initiated")
      
      self.img = Noneself.bridge = CvBridge()
      self.sub = self.create_subscription(Image,"/stream",self.camara_callback,10)
      self.timer = self.create_timer(0.05,sekf.timer_callback)

      def camara_callback(self,msg):
         self.img = self.bridge.imgmsg_to_cv2(msg,"bgr8") #bgr8 mono8 para b/n y bgr para color
      
      def timer_callback(self):
         if self.img is not None:
            cv2.imshow("Received image",self.img)
            cv2.waitKey(1)
    
      
      
   
  

     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = OpenCVBridge()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
