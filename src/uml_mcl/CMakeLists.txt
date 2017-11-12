cmake_minimum_required(VERSION 2.4.6)
project(uml_mcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  stage_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uml_race_catkin
#  CATKIN_DEPENDS stage
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


