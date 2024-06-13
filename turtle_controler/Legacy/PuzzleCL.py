#! /usr/bin/env python3
import rclpy,time,math,sys # type: ignore
from rclpy.node import Node# type: ignore
from geometry_msgs.msg import Twist# type: ignore
from turtlesim.msg import Pose# type: ignore
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
      
      self.pose = Pose()
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
      
      #Diferentes trayectorias
      self.lista = [(0,1.5),(1.5,1.5),(1.5,0),(0,0)] #Cuadrado
      #self.lista = [(0,1),(1,1),(0,0)] # Triangulo
      #self.lista = [(0.5,0.5),(0,1),(-0.5,0.5),(0,0)] # Rombo
      #self.lista = [(1.5,0),(0,1),(-1.5,0.0),(-1,-2),(1,-2),(1.5,0)] # Pentagono
   
   def callback_turtle_pose(self,msg):
      #Se recibe la pose del robot del node de odometría
      self.pose = msg
   
   def callback_light(self,msg):
      self.speed = msg.data   

      
   # Función go to point simple
   def go_to_point(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            #calculo del error
            Dx, Dy = target_x-self.pose.x, target_y-self.pose.y
            e_dist = math.sqrt(Dx**2 + Dy**2)
            #Checa si la posición esta dentro del limite de tolerancia
            if abs(e_dist) <= self.linTol: self.pub.publish(Twist()); print("Target reached") ;break
            #calculo de error angular
            e_ang = math.atan2(Dy,Dx) - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))

            
            #Publicación de mensajes
            msg.linear.x = max(min(self.Kv * e_dist,self.linMax),-self.linMax)
            msg.angular.z = max(min(self.Ka * a_ang,self.angMax),-self.angMax)
            print("L: " + str(e_dist))
            print("A: "+ str(a_ang))
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)
         
   def go_to_angle(self,target_theta):
      msg = Twist()
      while True:
         if self.pose is not None:

            #Calculo de error angular
            e_ang = target_theta - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))
            #print(a_ang)
            #print(self.pose)
            #print(a_ang)

            #Checa si la posición esta dentro del limite de tolerancia
            if abs(a_ang) <= self.angTol: 
               self.pub.publish(Twist())
               print("Target reached")
               break
            #print(a_ang)
            #Envía velocidad a robot

            #control de velocidad
            if(self.envelope < self.speed ):
               self.envelope = self.envelope + 0.02/self.accTime

            if (self.envelope > self.speed):
               self.envelope = self.envelope - 0.02/self.accTime

            msg.angular.z = max(min(self.Ka * a_ang,self.angMax),-self.angMax)*self.envelope
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)
   
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

      # Función go to point simple
   def acc_go_to_point(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            #calculo del error
            Dx, Dy = target_x-self.pose.x, target_y-self.pose.y
            e_dist = math.sqrt(Dx**2 + Dy**2)
            #Checa si la posición esta dentro del limite de tolerancia
            if abs(e_dist) <= self.linTol: 
               self.pub.publish(Twist())
               print("Target reached")
               self.envelope = 0.0
               break
            #calculo de error angular
            e_ang = math.atan2(Dy,Dx) - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))

            #control de velocidad
            if(self.envelope < self.speed ): 
               self.envelope = self.envelope + 0.02/self.accTime

            if (self.envelope > self.speed):
               self.envelope = self.envelope - 0.02/self.accTime
            #Publicación de mensajes
            msg.linear.x = max(min(self.Kv * e_dist,self.linMax),-self.linMax)*self.envelope
            msg.angular.z = max(min(self.Ka * a_ang,self.angMax),-self.angMax)* self. envelope

            #print("L: " + str(e_dist))
            #print("A: "+ str(a_ang))
            #print("---------------------")
            #print("V: " + str(msg.linear.x))
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)

 

   def angleandpursuit(self,target_x,target_y):
      msg = Twist()
      self.pub.publish(msg)
      time.sleep(2)
      #print("Target: x: " + str(target_x) + " y: " + str(target_y) )
      #print("yes")
      actual_x = self.pose.x
      actual_y = self.pose.y

      #calcula angulo para función go to angle
      errx = target_x-actual_x
      erry = target_y-actual_y

      angle = math.atan2(erry,errx)
      self.go_to_angle(angle)
      time.sleep(2)
      self.acc_go_to_point(target_x,target_y)
      #print("Target Reached")
      time.sleep(2)


    

   def principal(self):
      print("begin...")
      time.sleep(4)
      for coo in self.lista:
         self.angleandpursuit(coo[0],coo[1])
      self.get_logger().info("Finished")

     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: nodeh.principal()
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
