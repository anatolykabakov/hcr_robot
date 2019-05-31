cmake_minimum_required(VERSION 2.8.3)
project(online_slam)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
