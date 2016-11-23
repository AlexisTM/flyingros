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

Actual goals:

* Indoor Navigation (Pozyx -Decawave-, Camera, Lasers)
* Outdoor Navigation (Camera, RTK-GPS)
* Payloads 

Future goals:

* Object tracking 
* SLAM
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
  - [x] lasers : *Replaced by `flyingros_pose/cfg/laser/2lasers.yaml & 6lasers.yaml`*
  - [x] taskController
  - [x] UAV : *Test the raw_setpoint to takeoff from offboard*
  - [x] task : *easy to add new tasks*
    * ARM : *to be reviewed, change timeout method*
    * DISARM : *to be reviewed, change timeout method*
    * INIT_UAV : *to be reviewed, change timeout method*
    * LOITER
    * TAKEOFF : **TO BE TESTED**
    * LAND : **TO BE TESTED**
    * TARGET
    * GRAB : **TO BE IMPLEMENTED**
* flyingros_nav
  - [x] Task_node : Start & manage the task controller *missing get & send current task*
  - [ ] Nav\_application : Using task\_node, it adds the ability to stop the task spinning and take back manual control over the multicopter  **Work in progress**
  - [ ] Manual_node : bypass the controller 
    individual tasks working well
* flyingros_pose :  *To be reviewed & tested*
  - [x] laser_altitude *To be tested*
  - [x] rtk *To be tested*
  - [x] six_lasers *To be tested*
* flyingros_web :  *To be reviewed & tested*
  - [x] web_export
  - [x] website
* [AlexisTM/ethzasl_msf](https://github.com/AlexisTM/ethzasl_msf) 
  - [x] dual_position (rtk + lasers(position))
  - [x] pose\_position\_pressure (camera + rtk + lasers(altitude))
  - [x] position\_pressure (rtk + lasers(altitude))

#### Do not forget tasks 

* Verify convention on Laser algorithms