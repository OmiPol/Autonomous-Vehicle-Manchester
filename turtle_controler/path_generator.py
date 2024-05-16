#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_msgs.msg import Float32, Bool


class Path(Node):
   def __init__(self):
      
      super().__init__("Path_generator") #declara el nodo
      self.get_logger().info("Node initiated")
      self.pub = self.create_publisher(Pose, "/point",1)
      self.sub = self.create_subscription(Bool,"/acknowledged",self.Path_callback,1)
      
      self.list_counter = 0
      #self.lista = [(1.0,0.0),(1.0,1.0),(0.0,1.0),(0.0,0.0)]
      #self.lista = [(1.0,0.0),(1.0,2.0),(0.0,0.0)]
      self.lista = [(6.0,6.0),(7.0,6.0),(7.0,7.0),(6.0,6.0)]

   
   def Path_callback(self,msg):
      msg = Pose()
      print("called")
      
      self.pub.publish(msg)
      if(self.list_counter >= len(self.lista)):
         msg.x = self.lista[self.list_counter][0]
         msg.y = self.lista[self.list_counter][1]
         self.pub.publish(msg)
            
      else:
         msg.x = self.lista[self.list_counter][0]
         msg.y = self.lista[self.list_counter][1]
         self.pub.publish(msg)
         self.list_counter = self.list_counter +1
      
      
      
      

      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Path()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
