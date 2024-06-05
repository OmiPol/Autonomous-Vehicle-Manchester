#! /usr/bin/env python3
import rclpy,time,math,sys # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from turtlesim.msg import Pose # type: ignore
from std_msgs.msg import Float32, Bool, String

#HOLIIIICANOLIIII

class SM(Node):
    def __init__(self):
      
      super().__init__("Master_State_Machine") #declara el nodo
      self.get_logger().info("MSM Node initiated ")
      
      #Creación de subscriptores y publicadores
      self.pub = self.create_publisher(String, "/master_state", 1)
      
      self.strl = self.create_subscription(String,"/state_streetlight",self.callback_light,1)
      self.turnsign = self.create_subscription(String,"/turn_sign",self.callback_turn,1)
      self.warningsign = self.create_subscription(String,"/warn_sign",self.callback_warn,1)
      self.lineas = self.create_subscription(String,"/lineas",self.callback_lineas,1)
      
      self.callback_time = 0.08
      self.state_machine = self.create_timer(self.callback_time,self.machine_callback)
      
      
      #Inicia espacios de memoria
      self.light = "green"
      self.turn = "ahead_only"
      self.warn_sign = "null"
      self.lineas = "linea"
      
      
      self.state = "start"
      #Estados posibles
      #start --
      #seguir_linea--
      #atiende_zebra--
      #espera --

      #cruza_crucero_right_ign_ln --
      #cruza_crucero_left_ign_ln --
      #cruza_crucero_straight_ign_ln  

      #cruza_crucero_right --
      #cruza_crucero_left --
      #cruza_crucero_straight --
      
      #slow--
      #give_way --
      #seguir_ign_gw--
      #end--
      
      #Contador
      self.contador = 0.0
      
    #Callbacks de obtención de datos  
    def callback_light(self,msg):
        self.light = msg.data
    
    def callback_turn(self,msg):
        self.turn = msg.data
    
    def callback_warn(self,msg):
        self.warningsign = msg.data

    def callback_lineas(self,msg):
        self.lineas = msg.data
        #print("linea")
        
   
    def machine_callback(self):
        msg = String()
        msg.data = self.state
        self.pub.publish(msg)
        
        if(self.state == "start"):
            self.state = "seguir_linea"
            return   
        
        #CRUCERO DE ZEBRA    
        if (self.state == "seguir_linea" and self.lineas == "zebra"):
            self.state = "atiende_zebra"
            return
        
        if (self.state == "slow" and self.lineas == "zebra"):
            self.state = "atiende_zebra"
            return
        
        if (self.state == "seguir_ign_gw" and self.lineas == "zebra"):
            self.state = "atiende_zebra"
            return
        
        
        if (self.state == "atiende_zebra" and (self.light == "red" or self.light == "yellow")):
            self.state = "espera"
            return
        
        if (self.state == "espera" and (self.light == "red" or self.light == "yellow")):
            self.state = "espera"
            return
        
        #ESTADOS DE AUTONOMÍA IGNORANDO LINEAS
        
        #Casos de Right
        if (self.state == "espera" and self.light == "green" and self.turn == "turn_right_ahead" ):
            self.state = "cruza_crucero_right_ign_ln"
            return
        
        if (self.state == "atiende_zebra" and self.light == "green" and self.turn == "turn_right_ahead"):
            self.state = "cruza_crucero_right_ign_ln"
            return
        
        #Casos de Straight Ahead
        if (self.state == "espera" and self.light == "green" and self.turn == "ahead_only" ):
            self.state = "cruza_crucero_straight_ign_ln"
            return
        
        if (self.state == "atiende_zebra" and self.light == "green" and self.turn == "ahead_only"):
            self.state = "cruza_crucero_straight_ign_ln"
            return

        #Casos de Left
        if (self.state == "espera" and self.light == "green" and self.turn == "turn_left_ahead" ):
            self.state = "cruza_crucero_left_ign_ln"
            return
        
        if (self.state == "atiende_zebra" and self.light == "green" and self.turn == "turn_left_ahead"):
            self.state = "cruza_crucero_left_ign_ln"
            return
        
        #Timers para tiempo de autonomía

        if (self.state == "cruza_crucero_straight_ign_ln" and self.contador < 5):
            self.contador = self.contador + self.callback_time
            return

        if (self.state == "cruza_crucero_straight_ign_ln" and self.contador >= 5):
            self.state = "cruza_crucero_straight"
            return

        if (self.state == "cruza_crucero_right_ign_ln" and self.contador < 5):
            self.contador = self.contador + self.callback_time
            return

        if (self.state == "cruza_crucero_right_ign_ln" and self.contador >= 5):
            self.state = "cruza_crucero_right"
            return
        
        if (self.state == "cruza_crucero_left_ign_ln" and self.contador < 5):
            self.contador = self.contador + self.callback_time
            return

        if (self.state == "cruza_crucero_left_ign_ln" and self.contador >= 5):
            self.state = "cruza_crucero_left"
            return

        

        #ESTADO DE AUTONOMÍA BUSCANDO LINEAS


        #Regreso a seguidor de linea
        
        if (self.state == "cruza_crucero_left" and self.lineas == "linea" ):
            self.state = "seguir_linea"
            return
        
        if (self.state == "cruza_crucero_right" and self.lineas == "linea" ):
            self.state = "seguir_linea"
            return
        
        if (self.state == "cruza_crucero_straight" and self.lineas == "linea" ):
            self.state = "seguir_linea"
            return
        
        #Estados de Alerta
        
        if (self.state == "seguir_linea" and self.warn_sign == "roadwork_ahead" ):
            self.state = "slow"
            return
        
        if (self.state == "slow" and self.contador < 3):
            self.contador = self.contador + self.callback_time
            return
        
        if (self.state == "slow" and self.contador >= 3):
            self.contador = 0.0
            self.state = "seguir_linea"
            return
            
        if (self.state == "seguir_linea" and self.warn_sign == "give_way"):
            self.state = "give_way"
            return
        
        if (self.state == "give_way" and self.contador < 3):
            self.contador = self.contador + self.callback_time
            return
        
        if (self.state == "give_way" and self.contador >= 3):
            self.contador = 0.0
            self.state = "seguir_ign_gw"
            return
        
        if (self.state == "seguir_ign_gw" and self.contador < 4):
            self.contador = self.contador + self.callback_time
            return
        
        if (self.state == "seguir_ign_gw" and self.contador >= 4):
            self.contador = 0.0
            self.state = "seguir_linea"
            return
        #Estado final
        if (self.state == "seguir_linea" and self.warn_sign == "stop"):
            self.state = "end"
            return
        
        if (self.state == "slow" and self.warn_sign == "stop"):
            self.state = "end"
            return
        
        if (self.state == "seguir_ign_gw" and self.warn_sign == "stop"):
            self.state = "end"
            return
            
            
        
        
        
        
        
        
        
##########################################################################################

     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = SM()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
