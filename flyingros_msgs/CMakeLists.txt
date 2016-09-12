cmake_minimum_required(VERSION 2.8.3)
project(flyingros_msgs)

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
   Report.msg
   RPY.msg
   RPYPose.msg
   Battery.msg
   Mission.msg
   Task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
)


catkin_package(
  CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs geometry_msgs
)
