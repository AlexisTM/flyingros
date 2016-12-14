FlyingROS [![Build Status](https://api.travis-ci.org/AlexisTM/flyingros.svg?branch=master)](https://travis-ci.org/AlexisTM/flyingros)
==============

Around the Internet of robotics, there is a lot of open-source/open-hardware projects to make an awesome flying robot : **ROS**, **MAVLink**, **PX4**, **Mavros**, **Odroid XU4**. Multiple companies uses thoses projects to make money without contributing to the community. That's why FlyingROS comes to life. To bring user an ***easy*** way to fly multicopters with all tools incorporated together.

Flying Robot Operating System is designed to be the main **OPEN-SOURCE** package for your multicopter consumer application or closed-source commercial application.

The way the packages are organized are available in the [Project architecture](PROJECT_ARCHITECTURE.MD) file.

Actual status
------------

Experimental, do not use unless you want to develop with us.

Hardware
------------

* PixHawk (200$)
* Odroid XU4 (79$ naked)
* WiFi antenna (connect to external hotspot) or Modem (create an AP)
* The localisation system you want (see the [flyingros_pose package](flyingros_pose))

Software
-----------

* ROS
* PX4
* MAVLink
* Mavros

Goals
------------

Actual goals:

* Indoor Navigation (Pozyx -Decawave-, Camera, Fixed lasers)
* Outdoor Navigation (Camera, RTK-GPS)
* Payloads (different PID configurations depending on the weigth of the multicopter, what it lifts)

Future goals:

* Object tracking 
* SLAM
* Simulation

In depth status
------------
* `flyingros_libs/tasks.py` : Tested
    * Can be used as python scenarios or using the task node (preferred)
    * Can use position setpoints and raw setpoints (not tested on 1.5, was not working on 1.3 Firmware but should be working by now).
    * TODO 
        * Update tasks to avoid the use of sleep function
* `flyingros_nav/task_node` : Tested 
    * Able to manage tasks using Tasks.py library, from ROS and the web interface
    * Provide the current task
    * Can be paused (To be implemented in the web interface)
    * TODO 
        * add a Try Catch to avoid wrong data from user (to be implemented in )
* `flyingros_nav/control_thread.py` : Tested
    * "Manual" offboard control : (control)
* `flyingros_pose/six_lasers_algorithm`
    * Working well
* `flyingros_web/www` : 
    * TODO 
        * Allow to pause/unpause the task execution
        * Add a emergency button

