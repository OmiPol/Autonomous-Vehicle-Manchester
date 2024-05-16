#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Controler(Node):
   def __init__(self):
      
      super().__init__("controller") #declara el nodo
      self.get_logger().info("Node initiated")
      self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)

      self.posx = 5.5
      self.posy = 5.5
      self.head = 0.0
      
      self.lista = [(5,5),(5,7),(7,1),(1,1)]
      
   def rotation(self,desired_point):
      desx,desy = desired_point
      message = Twist()
      x = desx-self.posx
      y = desy-self.posy
      ang = math.atan2(math.sin(y),math.cos(x))
      dif = ang - self.head
      message.angular.z = dif
      self.pub.publish(message)
      time.sleep(1)
      self.head = ang
  
         
      
   def advance(self, desired_point):
      desx,desy = desired_point
      message = Twist()
      x = desx-self.posx
      y = desy-self.posy
      dist = math.sqrt(x**2+y**2)
      message.linear.x = dist
      self.pub.publish(message)
      self.posx=desx
      self.posy=desy
      time.sleep(1)
   
   def prueba(self):
      message = Twist()
      message.angular.z= 2* math.pi
      self.pub.publish(message)
      time.sleep(3)
   
   def principal(self):
   
     time.sleep(2)
     #self.prueba()
     while(True):
        for indice in self.lista:
        
           self.rotation(indice)
           #time.sleep(1)
           self.advance(indice)
           #time.sleep(1)
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: nodeh.principal()
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
