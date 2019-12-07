# hcr robot ros node


roslaunch robot_node bringup.launch robot_ns:=robot_0 robot_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0

roslaunch robot_node bringup.launch robot_ns:=robot_1 robot_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel

export TURTLEBOT3_MODEL=waffle_pi
ROS_NAMESPACE=robot_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=robot_0/base_link set_odom_frame:=robot_0/odom set_map_frame:=robot_0/map

ROS_NAMESPACE=robot_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=robot_1/base_link set_odom_frame:=robot_1/odom set_map_frame:=robot_1/map

roslaunch turtlebot3_nps multi_map_merge.launch