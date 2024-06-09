#! /usr/bin/env python3
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control
import rclpy,time,math,sys # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from turtlesim.msg import Pose # type: ignore
from std_msgs.msg import Float32, Bool, Int32, String


class Controler(Node):
   def __init__(self):
      
      super().__init__("Controler_CL") #declara el nodo
      self.get_logger().info("Node initiated")
      #Creación de subscriptores y publicadores
      self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
      self.sub= self.create_subscription(Pose,"/pose",self.callback_turtle_pose,1)
      self.line = self.create_subscription(Int32, "/line_error",self.callback_line,1)
      self.state = self.create_subscription(String,"/master_state",self.callback_state,1)
      self.fuzz = self.create_timer(0.02,self.master_control)
      #self.timer = self.create_timer(0.02,self.callback_envelope)
      
      #Control de velocidad
      self.envelope = 0.0 #Multiplica la salida de velocidad
      #self.speed = 0.0 #Define el valor de envelope
      #self.tiempo = 1.0 #define tiempo a alcanzar siguiente valor
      #self.interval = 0.02 #tiempo de llamado de callback
      #self.adder = 0.2
      #self.flag = True #una bandera idk
      #self.thresh = 0.03 #Threshold

      #Estado
      self.state = "start"
      
      self.pose = None
      #Valores de constantes proporcionales
      self.Kv = 0.15
      self.Ka = 0.85 #anterior 0.7

      self.L = 0.66
      #Parametros de velocidades máximas
      self.linMax = 0.1
      self.angMax = 0.7
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

      self.LinV['Poca']=fuzz.zmf(self.LinV.universe,3,6)#original 0,3
      self.LinV['Media']=fuzz.trimf(self.LinV.universe,[3,5,7]) #original 2,4,6
      self.LinV['Alta']=fuzz.smf(self.LinV.universe,5,10) #orignal 5,10

      self.AngV['Muy Izquierda']=fuzz.zmf(self.AngV.universe,-10,-4)
      self.AngV['Izquierda']=fuzz.trimf(self.AngV.universe,[-6,-5,-2]) #orignal -5, -4, -1
      self.AngV['Cero']=fuzz.trimf(self.AngV.universe,[-2,0,2])
      self.AngV['Derecha']=fuzz.trimf(self.AngV.universe,[2,5,6]) #original 1,4,5
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

   def callback_line(self,msg):
      #Recibe dato de posición de la linea
      self.linpos = msg.data #Actuizar dato

   def callback_state(self,msg):
      self.state = msg.data
      #self.get_logger().info(str(self.state))
    
   #def callback_envelope(self):
      #dif = self.speed - self.envelope
      # if(abs(dif) >= self.thresh and self.flag == True):
      #         iteraciones= self.tiempo/self.interval
      #         self.adder = (dif)/ iteraciones
      #         self.flag = False
      #         #print(self.adder)
      # if(abs(dif)<= self.thresh):
      #   self.flag = True
      #   #print("true")
      # else:
      #   self.envelope = self.envelope + self.adder
      # self.get_logger().info(str(self.envelope))
      #self.envelope = self.speed
        
   
   def master_control(self):
      
      if (self.state == "start" or self.state == "give_way" or self.state == "ended"):
         #self.get_logger().info(str(self.state))
         msg = Twist()
         self.pub.publish(msg)
         return
      
      if(self.state == "seguir_linea" or self.state == "seguir_ign_gw"):
         self.envelope = 1.0
         self.line_control()
         return

      if(self.state == "llega_a_zebra"):
         self.envelope = 0.6
         self.move_straight()
         
      if(self.state == "atiende_zebra"):
         self.envelope = 1.0
         self.move_straight()
         return
      
      if(self.state=="espera"):
         self.envelope = 0.0
         self.move_straight()
         return
      
      if(self.state == "cruza_crucero_straight" or self.state == "cruza_crucero_straight_ign_ln"):
         self.envelope = 0.7
         self.move_straight()
         return
      
      if(self.state == "cruza_crucero_left" or self.state == "cruza_crucero_left_ign_ln"):
         self.envelope = 0.7
         self.move_left()
         return

      if(self.state == "cruza_crucero_right" or self.state == "cruza_crucero_right_ign_ln"):
         self.envelope = 0.7
         self.move_right()
         return
      
      
      if(self.state == "slow"):
         self.envelope = 0.4
         self.line_control
         return

      if(self.state == "ending"):
         self.envelope=0.6
         self.line_control()
      
      

         
      
         
         
      
   def line_control(self):
      
      #print("a")
      msg = Twist()
      if(self.linpos != 69420):
         self.lineFollow.input['Cent']= self.linpos#self.linpos #enviar datos a fuzzy
      
         self.lineFollow.compute() 
      
         Lineal = (self.lineFollow.output['LinV'])/10 *self.linMax *self.envelope #
     
         Angular = (self.lineFollow.output['AngV'])/10 *self.angMax*self.envelope #

         #self.get_logger().info(f"Lineal: {Lineal}")
         #self.get_logger().info(f"Angular: {Angular}")
      
      
         msg.linear.x = max(min(Lineal,self.linMax),-self.linMax) #lineal v
         msg.angular.z = max(min(Angular,self.angMax),-self.angMax) # anular w
      self.pub.publish(msg)
    
   def move_straight(self):
      msg = Twist()
      msg.linear.x = self.linMax * self.envelope
      self.pub.publish(msg)#lineal v
   #MODIFICADAS MOVE_LEFT Y MOVE_RIGHT
   def move_left(self):
      msg = Twist()
      msg.linear.x = self.linMax * 0.7
      msg.angular.z = self.angMax *0.3
      self.pub.publish(msg)

   def move_right(self):
      msg = Twist()
      msg.linear.x = self.linMax * 0.7
      msg.angular.z = self.angMax *-0.3
      self.pub.publish(msg)
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
