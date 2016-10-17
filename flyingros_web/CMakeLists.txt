cmake_minimum_required(VERSION 2.8.3)
project(flyingros_web)

find_package(catkin REQUIRED COMPONENTS
  rospy)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flyingros_web
  CATKIN_DEPENDS flyingros_libs flyingros_msgs rosbridge_server mavros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/nodes/*)
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)

#############
## Testing ##
#############
