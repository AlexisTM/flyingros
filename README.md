FlyingROS [![Build Status](https://api.travis-ci.org/AlexisTM/flyingros.svg?branch=master)](https://travis-ci.org/AlexisTM/flyingros)
=================

Around the Internet of robotics, there are a lot of open-source/open-hardware projects to make an awesome flying robot : **ROS**, **MAVLink**, **PX4**, **Mavros**, **Odroid XU4**. Multiple companies uses thoses projects to make money without contributing to the community. This is an example of application. As it is no more maintained, use it with care but could could be taken as an example to make yours.

The way the packages are organized are available in the [Project architecture](https://github.com/AlexisTM/flyingros/blob/master/PROJECT_ARCHITECTURE.MD) file.

Hardware
------------

* PixHawk (200$)
* Odroid XU4 (79$ naked)
* WiFi antenna (connect to external hotspot) or Modem (create an AP)
* The localisation system you want (see the [flyingros_pose package](flyingros_pose))

Software used
-----------

* ROS
* PX4
* MAVLink
* Mavros

List of things to salvage
-------------------------

* [Motor testing through ESC & Arduino](https://github.com/AlexisTM/flyingros/tree/master/flyingros/external/esc_control)
* [Pixhawk configuration **(Outdated)**](https://github.com/AlexisTM/flyingros/blob/master/flyingros/SPECIFIC_PIXHAWK.md)
* [Odroid configuration (or RPi)](https://github.com/AlexisTM/flyingros/blob/master/flyingros/SPECIFIC_ODROID.md)
* [Basic keyboard teleop for PX4](https://github.com/AlexisTM/flyingros/blob/master/flyingros_nav/nodes/control_thread.py)
* [Example of task-based PX4 control](https://github.com/AlexisTM/flyingros/tree/master/flyingros_nav) composed of:
  * [tasks.py](https://github.com/AlexisTM/flyingros/blob/master/flyingros_libs/src/flyingros_libs/tasks.py): Abstraction to high level tasks (move, loiter, arm, disarm, takeoff) see the [Documentation](https://github.com/AlexisTM/flyingros/blob/master/flyingros_nav/TASKS.MD) and the [usage](https://github.com/AlexisTM/flyingros/blob/master/flyingros_nav/scenari/scenari_py/scenario_circle)
  * [task_node](https://github.com/AlexisTM/flyingros/blob/master/flyingros_nav/nodes/task_node): a controller spinning through tasks
  * [nav_application](https://github.com/AlexisTM/flyingros/blob/master/flyingros_nav/nodes/nav_application): An urwid application to manage tasks
  * [flyingros_web](https://github.com/AlexisTM/flyingros/tree/master/flyingros_web): A basic web app to manage tasks via websocket and rosbridge
* [flyingros_pose](https://github.com/AlexisTM/flyingros/tree/master/flyingros_pose) lots of information about SLAM & visual odometry.
