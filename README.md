# hcr robot ros node

ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_slam gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map

roslaunch robot_node bringup.launch robot_ns:=robot_0 robot_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0

roslaunch robot_node bringup.launch robot_ns:=robot_1 robot_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel
