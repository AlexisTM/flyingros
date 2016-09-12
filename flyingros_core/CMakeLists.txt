cmake_minimum_required(VERSION 2.8.3)
project(flyingros_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  flyingros_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserpack
CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#include_directories(
#  include
#  ${mavlink_INCLUDE_DIRS}
#  ${console_bridge_INCLUDE_DIRS}
#)

## Declare a cpp library
#add_library(mavconn
#  src/mavlink_helpers.cpp
#  src/interface.cpp
#  src/serial.cpp
#  src/udp.cpp
#  src/tcp.cpp
#)
