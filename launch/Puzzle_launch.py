#!/use/bin/env python3
import os 
from ament_index_python import get_package_share_directory # type: ignore
from launch import LaunchDescription # type: ignore
from launch.actions import IncludeLaunchDescription # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
   odometry = Node(
      package='turtle_controler',
      executable = 'odometry',
      output = 'screen',)
   
   Semaforo = Node(
      package='turtle_controler',
      executable = 'Semaforo',
      output = 'screen',)
   
   PuzzleSM = Node(
      package='turtle_controler',
      executable = 'PuzzleSM',
      output = 'screen',)   
   
   #PuzzleCL = Node(
    #  package='turtle_controler',
     # executable = 'PuzzleCL',
      #output = 'screen',)

   PuzzleLineCL = Node(
      package='turtle_controler',
      executable = 'PuzzleLineCL',
      output = 'screen',)
   
   Linea = Node(
      package='turtle_controler',
      executable = 'Linea',
      output = 'screen',)
      
   l_d = LaunchDescription([odometry, Semaforo, PuzzleSM, PuzzleLineCL, Linea])
   return l_d
   
