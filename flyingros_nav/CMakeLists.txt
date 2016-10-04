cmake_minimum_required(VERSION 2.8.3)
project(flyingros_nav)

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
  CATKIN_DEPENDS flyingros_libs flyingros_msgs mavros_msgs
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

install(DIRECTORY scenari/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenari
)

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

#############
## Testing ##
#############
