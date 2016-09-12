cmake_minimum_required(VERSION 2.8.3)
project(flyingros_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
)

catkin_python_setup()

add_message_files(
   FILES
   Distance.msg
   Req_reset.msg
   Report.msg
   RPY.msg
   RPYPose.msg
   Battery.msg
   Reports.msg
   Mission.msg
   Task.msg
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
)


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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
#install(TARGETS mavconn
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/mavconn/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mavconn
#  FILES_MATCHING PATTERN "*.h"
#)

## Install cmake files (thanks to cmake_modules package)
#install(DIRECTORY cmake/Modules
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
#)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)

#catkin_add_gtest(mavconn-test test/test_mavconn.cpp)
#target_link_libraries(mavconn-test mavconn)

#endif()

# vim: ts=2 sw=2 et:
