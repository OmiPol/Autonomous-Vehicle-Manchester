#! /usr/bin/env python3
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control
import rclpy,time,math,sys # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from turtlesim.msg import Pose # type: ignore
from std_msgs.msg import Float32, Bool, Int32


class Controler(Node):
   def __init__(self):
      
      super().__init__("Controler_CL") #declara el nodo
      self.get_logger().info("Node initiated")
      #Creación de subscriptores y publicadores
      self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
      self.sub= self.create_subscription(Pose,"/pose",self.callback_turtle_pose,1)
      self.ack = self.create_publisher(Bool,"acknowledged",1)
      self.strl = self.create_subscription(Float32,"/states",self.callback_light,1)
      self.line = self.create_subscription(Int32, "/line_error",self.callback_line,1)
      
      self.fuzz = self.create_timer(0.02,self.callback_fuzzy)
      
      self.envelope = 0.0
      self.speed = 0.0
      
      self.pose = None
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
      

      #FUZZY
      
      self.linpos = 0
      
      #Centrado de la linea #negativos izq positivos der
      self.Cent= control.Antecedent(np.arange(-159,160),'Cent')

      #LinealVel
      self.LinV=control.Consequent(np.arange(0,10),'LinV')

      #Porcentaje de propina
      self.AngV= control.Consequent(np.arange(-10,10),'AngV')


      self.Cent['Muy Izquierda']= fuzz.zmf(self.Cent.universe,-159,-50)
      self.Cent['Izquierda']=fuzz.trimf(self.Cent.universe,[-80,-60,-20])
      self.Cent['Centro']=fuzz.trimf(self.Cent.universe,[-30,0, 30])
      self.Cent['Derecha']=fuzz.trimf(self.Cent.universe,[20,60, 80])
      self.Cent['Muy Derecha']=fuzz.smf(self.Cent.universe,50,160)

      self.LinV['Poca']=fuzz.zmf(self.LinV.universe,0,3)
      self.LinV['Media']=fuzz.trimf(self.LinV.universe,[2,4,6])
      self.LinV['Alta']=fuzz.smf(self.LinV.universe,5,10)

      self.AngV['Muy Izquierda']=fuzz.zmf(self.AngV.universe,-10,-4)
      self.AngV['Izquierda']=fuzz.trimf(self.AngV.universe,[-5,-4,-1])
      self.AngV['Cero']=fuzz.trimf(self.AngV.universe,[-2,0,2])
      self.AngV['Derecha']=fuzz.trimf(self.AngV.universe,[1,4,5])
      self.AngV['Muy Derecha']=fuzz.smf(self.AngV.universe,4,10)

      self.r1 = control.Rule(self.Cent['Muy Izquierda'],(self.LinV['Poca'],self.AngV['Muy Derecha']))
      self.r2 = control.Rule(self.Cent['Izquierda'],(self.LinV['Media'],self.AngV['Derecha']))
      self.r3 = control.Rule(self.Cent['Centro'],(self.LinV['Alta'],self.AngV['Cero']))
      self.r4 = control.Rule(self.Cent['Derecha'],(self.LinV['Media'],self.AngV['Izquierda']))
      self.r5 = control.Rule(self.Cent['Muy Derecha'],(self.LinV['Poca'],self.AngV['Muy Izquierda']))
      
      self.Follower = control.ControlSystem([self.r1,self.r2,self.r3,self.r4,self.r5])
      
      self.lineFollow = control.ControlSystemSimulation(self.Follower)

   
   def callback_turtle_pose(self,msg):
      #Se recibe la pose del robot del node de odometría
      self.pose = msg
   
   def callback_light(self,msg):
      self.speed = msg.data   
      
   def callback_line(self,msg):
      
      self.linpos = msg.data #Actuizar dato
      
   def callback_fuzzy(self):
      
      #print("a")
      msg = Twist()
      if(self.linpos != 69420):
         self.lineFollow.input['Cent']= self.linpos#self.linpos #enviar datos a fuzzy
      
         self.lineFollow.compute() 
      
         Lineal = (self.lineFollow.output['LinV'])/10 *self.linMax #
     
         Angular = (self.lineFollow.output['AngV'])/10 *self.linMax
      
      
         msg.linear.x = max(min(Lineal,self.linMax),-self.linMax) #lineal v
         msg.angular.z = max(min(Angular,self.angMax),-self.angMax) # anular w
      self.pub.publish(msg)
    

      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
