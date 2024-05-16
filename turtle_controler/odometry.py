#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32


class Odometry(Node):
   def __init__(self):
      
      super().__init__("Odometry") #declara el nodo
      self.get_logger().info("Node initiated")
      
      qos_profile = QoSProfile(
         reliability = ReliabilityPolicy.BEST_EFFORT,
         durability = DurabilityPolicy.VOLATILE,
         depth = 1)
      
      
      self.pub = self.create_publisher(Pose, "/pose", 1)
      self.subR= self.create_subscription(Float32,"/VelocityEncR",self.callback_encR,qos_profile)
      self.subL= self.create_subscription(Float32,"/VelocityEncL",self.callback_encL,qos_profile)
      
      self.timer = self.create_timer(0.05,self.odometry_callback)
      
      self.wR = 0.0
      self.wL = 0.0
      self.x = 0.0
      self.y = 0.0
      self.theta =0.0
      self.R = 0.0505
      self.D = 0.17
      self.t0 = time.time()
      self.tact = 0.0
      
   def callback_encR(self,msg):  
      
      self.wR = msg.data
      
      
   def callback_encL(self,msg):  
      
      self.wL = msg.data
      
   def odometry_callback(self):
      msg = Pose()
      self.tact = time.time()
      delta = self.tact-self.t0
      #print(delta)
      #print("1")
      linealV = self.R*(self.wL+self.wR)/2.0
      #print("2")
      angularV =self.R*(self.wR-self.wL)/self.D
      
      
      theta = (self.theta) + (delta * angularV)     
      x = (self.x) + (delta * linealV * math.cos(theta))
      y = (self.y) + (delta * linealV * math.sin(theta))
      
      self.x = x
      self.y = y
      self.theta = theta
      
      msg.x = x
      msg.y = y
      msg.theta = theta
      self.pub.publish(msg)
      self.t0 = self.tact 
      
  
      
  

     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Odometry()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
