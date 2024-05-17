#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32, Bool


class Controler(Node):
   def __init__(self):
      
      super().__init__("Controler_CL") #declara el nodo
      self.get_logger().info("Node initiated")
      #Creación de subscriptores y publicadores
      self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
      self.sub= self.create_subscription(Pose,"/pose",self.callback_turtle_pose,1)
      self.ack = self.create_publisher(Bool,"acknowledged",1)
      self.strl = self.create_subscription(Float32,"/states",self.callback_light,1)
      self.envelope = 0.0
      self.speed = 0.0
      
      self.pose = None
      rclpy.spin_once(self)
      #Valores de constantes proporcionales
      self.Kv = 0.2
      self.Ka = 0.85 #anterior 0.7

      self.L = 0.66
      #Parametros de velocidades máximas
      self.linMax = 0.1
      self.angMax = 0.3
      #Tolerancias de error
      self.linTol = 0.05
      self.angTol = 0.05

      #Control de aceleración
      self.accTime = 0.5
      

   
   def callback_turtle_pose(self,msg):
      #Se recibe la pose del robot del node de odometría
      self.pose = msg
   
   def callback_light(self,msg):
      self.speed = msg.data   

      
 
   def pose_pursuit(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            #Calculo de errores
            diff_x, diff_y = target_x - self.pose.x, target_y - self.pose.y
            e_dist = math.sqrt(diff_x**2 + diff_y**2)

            #Checa si esta dentro de la tolerancia
            if abs(e_dist) < self.linTol: 
               self.pub.publish(Twist()); 
               #print("Target reached");
               break
            sq = math.sin(self.pose.theta)
            cq = math.cos(self.pose.theta)
            
            #Calcula velocidades
            w = ((diff_y * cq) - (diff_x * sq)) / self.L
            v = (diff_x + (self.L * w * sq)) / cq 

            #Envía mensaje a robót
            msg.linear.x = max(min(v,self.linMax),-self.linMax) #lineal v
            msg.angular.z = max(min(w,self.angTol),-self.angTol) # anular w
            self.pub.publish(msg)
            rclpy.spin_once(self) #checa mensajes
            time.sleep(0.02) #Ahorra ciclos de procesamiento
  	     #print("cerca cerquita vamos")



   
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: nodeh.principal()
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
