#! /usr/bin/env python3
import rclpy,time,math,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose
#from my_interfaces.msg import Pose

from std_msgs.msg import Float32, Bool


class Controler(Node):
   def __init__(self):
      
      super().__init__("controller") #declara el nodo
      self.get_logger().info("Node initiated")
      self.pub = self.create_publisher(Twist, "cmd_vel", 1)
      self.pub2 = self.create_publisher(Bool, "Acknowledged",1)
      self.sub = self.create_subscription(Pose,"Pose",self.LA_callback,1)

      self.posx = 0.0
      self.posy = 0.0
      self.head = 0.0 #math.pi/2 #Esto significa que el heading es hacia el frente
      
      self.pose = Pose()
      self.L = 0.66
      self.Pthresh= 0.1
      self.Kv = 2.0
      
      #self.lista = [(5,5),(5,7),(7,1),(1,1)]
    
    
   def LA_callback(self,msg):
      desired_Point = (msg.position.x,msg.position.y)
      
      self.get_logger().info("DP: "+str(desired_Point))
      self.rotation(desired_Point)
      self.advance(desired_Point)
      msg = Bool()
      msg.data = True
      self.pub2.publish(msg)
      self.get_logger().info("----------------------------------------------")
      
      
      
      
   def rotation(self,desired_point):
      vang = 0.4
      desx,desy = desired_point
      message = Twist()
      x = desx-self.posx
      y = desy-self.posy
      ang = math.atan2(y,x)#Esto esta maaal...
      err = ang - self.head      
      dif = math.atan2(math.sin(err),math.cos(err))
      #Para no matar la esp32 ni la Jetson lo cambie a que sea tiempo variable y velocidad fija...
      
      tiempo = (dif/vang)-0.4 #angulo / velocidad = tiempo #Entre 2 metodo heuristico para control de laso abierto :)
      
      message.angular.z = vang
      if(tiempo>0):
         self.pub.publish(message)
      
      self.get_logger().info("Tiempo angulo: "+str(abs(tiempo)) + " Diferencia angulo: " + str(dif))
      time.sleep(abs(tiempo))
      
      #Un segundito para difrenciar entre instrucciones
      message.angular.z = 0.0
      self.pub.publish(message)
      self.get_logger().info("Stop Angular")
      time.sleep(3)
      self.head = ang
      
  
         
      
   def advance(self, desired_point):
      vel = 0.2 #Cambiar velocidad aquí
      desx,desy = desired_point
      message = Twist()
      x = desx-self.posx
      y = desy-self.posy
      dist = math.sqrt(x**2+y**2) 
      
      # misma lógica que rotation, para no matar la jetson le voa dar una velocidad baja
      tiempo = dist / vel #  V=d/t -> V*t=d -> t = d/V
      message.linear.x = vel
      self.pub.publish(message)
      self.get_logger().info("Tiempo linear: "+str(abs(tiempo)) + " Distancia linear" + str(abs(dist)) )
      time.sleep(abs(tiempo))
      
      #Otro segundo de espera
      self.posx = desx
      self.posy = desy
      message.linear.x = 0.0
      self.pub.publish(message)

      time.sleep(3)
   
   def pose_pursuit(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            diff_x, diff_y = target_x - self.pose.x, target_y - self.pose.y
            e_dist = math.sqrt(diff_x**2 + diff_y**2)
            if abs(e_dist) < self.Pthresh: 
               self.pub.publish(Twist()); 
               print("Target reached");
               break
            sq = math.sin(self.pose.theta)
            cq = math.cos(self.pose.theta)
            
            w = ((diff_y * cq) - (diff_x * sq)) / self.L
            v = (diff_x + (self.L * w * sq)) / cq 
            msg.linear.x = v #liineal v
            msg.angular.z = w # anular w
            self.pub.publish(msg)
            rclpy.spin_once(self) #checa mensajes
            time.sleep(0.02) #Ahorra ciclos de procesamiento
  	     #print("cerca cerquita vamos")
  	     
   def go_to_angle(self,target_theta):
      msg = Twist()
      while True:
         if self.pose is not None:
            e_ang = target_theta - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))
            if abs(a_ang) <= 0.005: self.pub.publish(Twist()); print("Target reached") ;break
            print(a_ang)
            msg.angular.z = self.Kv * a_ang
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)

      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
