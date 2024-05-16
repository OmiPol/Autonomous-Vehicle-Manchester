#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32, Bool, String

#HOLIIIICANOLIIII
class SM(Node):
   def __init__(self):
      
      super().__init__("State_Machine") #declara el nodo
      self.get_logger().info("SM Node initiated ")
      self.timer = self.create_timer(0.05, self.callback_action)
      #Creación de subscriptores y publicadores
      self.pub = self.create_publisher(Float32, "/states", 1)
      self.strl = self.create_subscription(String,"/streetlight",self.callback_light,1)
      
      self.state = "start"
   
   def callback_light(self, msg):
      if self.state == "start" and msg.data == 'g':
         self.state = "green"
         #print("gee")

      elif self.state == "start" and msg.data == 'r':
         self.state = "red"
         #print("ree")

      elif self.state == "red" and msg.data == 'g':
         self.state = "green"
         #print("gee")

      elif self.state == "green" and msg.data == 'y':
         self.state = "yellow"
         #print("yee")
      
      elif self.state == "green" and msg.data == 'r':
         self.state = "red"
         #print("ree")

      elif self.state == "yellow" and msg.data == 'r':
         self.state = "red"
         #print("ree")


   def callback_action(self):
      msg = Float32()

      if self.state == "start":
         msg.data = 0.0
         self.pub.publish(msg) 

      elif self.state == "green":
         msg.data = 1.0
         self.pub.publish(msg)

      elif self.state == "yellow":
         msg.data = 0.5
         self.pub.publish(msg)

      elif self.state == "red":
         msg.data = 0.0
         self.pub.publish(msg)



     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = SM()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
