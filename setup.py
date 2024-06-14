import os
from glob import glob
#otro comentario

from setuptools import find_packages, setup

package_name = 'turtle_controler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        
        (os.path.join('share',package_name),glob('launch/*launch.[pxy][yam]*')),
        (os.path.join('share',package_name),glob('launch/*.[pxy][yam]*')),
        (os.path.join('share',package_name, 'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omi',
    maintainer_email='omi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #"controler = turtle_controler.controler:main",
        #"controller_PB = turtle_controler.controller_PB:main",
        #"path_generator = turtle_controler.path_generator:main",
        #"Turtle_CL = turtle_controler.Turtle_CL:main",
        #"odometry = turtle_controler.odometry:main",
        #"PuzzleCL = turtle_controler.PuzzleCL:main",
        #"Semaforo = turtle_controler.Semaforo:main",
        #"Linea = turtle_controler.Linea:main",
        #"PuzzleSM = turtle_controler.PuzzleSM:main",
        "PuzzleLineCL = turtle_controler.PuzzleLineCL:main",
        #"color_checker = turtle_controler.color_checker:main",
        "Linea2 = turtle_controler.Linea2:main",
        "PuzzleNNC = turtle_controler.PuzzleNNC:main",
        "master_sm = turtle_controler.master_sm:main",
        "NNC_prio = turtle_controler.NNC_prio:main",


        ],
    },
)
