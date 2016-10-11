cmake_minimum_required(VERSION 2.8.3)
project(flyingros_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest rosunit)
endif()

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

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test.task.py)
#  add_rostest(tests/your_second_rostest.test)
endif()
