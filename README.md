FlyingROS [![Build Status](https://api.travis-ci.org/AlexisTM/flyingros.svg?branch=master)](https://travis-ci.org/AlexisTM/flyingros)
==============

Around the Internet of robotics, there is a lot of open-source/open-hardware projects to make an awesome flying robot : **ROS**, **MAVLink**, **PX4**, **Mavros**, **Odroid XU4**. Multiple companies uses thoses projects to make money without contributing to the community. That's why FlyingROS comes to life. To bring user an ***easy*** way to fly multicopters with all tools incorporated together.

Flying Robot Operating System is designed to be the main **OPEN-SOURCE** package for your multicopter consumer application or closed-source commercial application.

Installation
------------

See [INSTALL](tree/master/INSTALL.md)

Actual status
------------

Experimental, do not use unless you want to develop with us (Thanks :D)

Hardware
------------

* PixHawk (200$)
* Odroid XU4 (79$ naked)
* WiFi antenna (connect to external hotspot) or Modem (create an AP)

Software
-----------

* ROS
* PX4
* MAVLink
* Mavros

Goals
------------

* Indoor Navigation
* Outdoor Navigation
* Payloads
* Object tracking
* SLAM
* RTK-GPS
* Simulation

In depth status
------------

* flyingros_msgs
  - [x] messages
    * Battery     (battery status)
    * Distance    (array of laser measures with the status **could change on demand**)
    * Mission     (Task array)
    * RPY         (Roll Pitch Yaw)
    * RPYPose     (Roll Pitch Yaw + X Y Z)
    * Report      (The whole UAV status)
    * Task        (A task the UAV has to do)
  - [x] services
    * MissionHandle  (send a mission & receive string)
    * MissionRequest (send string & receive mission)
    * TaskHandle     (send task & receive string)
    * TaskRequest    (send string & receive task)
* flyingros_libs  
  - [ ] lasers : *To be replaced by yaml config file*
  - [x] taskController
  - [ ] UAV : *To be tested & adapted*
  - [x] task : *easy to add new tasks*
    * ARM
    * DISARM
    * INIT_UAV
    * LOITER
    * TAKEOFF
    * LAND
    * TARGET
    * GRAB
* flyingros_nav
  - [x] Task_node : Start & manage the controller
  - [ ] Manual_node : bypass the controller **Work in progress**
  - [ ] Console_task : URWID interface for the Task Node
    individual tasks working well
* flyingros_nav :  *To be reviewed & tested*
  - [ ] laser_altitude
  - [ ] rtk
  - [ ] rtk_laser_fused
  - [ ] sex_lasers
* flyingros_web :  *To be reviewed & tested*
  - [ ] web_export
  - [ ] website 
