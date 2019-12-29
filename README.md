# hcr robot ros node

## Drivers

This repository contains the arduino driver and ROS (Melodic) node for wheeled robot, and some launch files for navigation stack ROS

## Usage
You can check this out into your catin workspace as follows:

    cd catkin_ws/src
    git clone https://github.com/anatolykabakov/robot.git
    cd catkin_ws
    catkin_make
    source devel/setup.bash

In robot

roslaunch robot_node bringup_group.launch robot_ns:=robot_0 robot_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0

or

roslaunch robot_node bringup.launch robot_ns:=robot_1 robot_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0

for teleop:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel


## TODO list

<!-- 1. Loading map in robot
1.1 Loading map using map_server
1.2 Loading map useing sevice from map server
2. Autonomus drive from point A to point B
2.1 Localization
2.1.1 Localization using offline map and amcl algoritm using inital pose
2.1.2 Localization using offline map and amcl algoritm using global lozalization service
2.2 Package move_base
2.2.1 Describe movebase params for this package

3. Driving amoung dynamic obstacles
3.1. Using local planner from move_base 

4. Send goal pose of robot from topological map in web interface

5. Debugging arduino driver for odometry -->


Чек-лист
1. Настройки сети
2. Точная одометрия робота
3. Точная локализация робота
4. Точное движение в точку 
5. Точное движение в точки для двух роботов.

