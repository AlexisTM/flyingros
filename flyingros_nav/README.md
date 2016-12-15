flyingros_nav
=============

flyingros_nav is the navigation package. This is still **experimental** and is subject to change (specifically ondemand, issues, etc.).


Navigation methods
-------------

* `rosrun flyingros_nav task_node` 
    * Uses tasks.py
    * ROS interface
    * Only sends setpoints
    * Works better with the web interface using rosbridge 
*  `rosrun flyingros_nav nav_application`
    * Uses tasks.py
    * Uses task_node (should be started first)
    * Runs on the odroid and uses URWID to give a SSH friendly interface 
* `rosrun flyingros_nav control_thread.py` 
    * Minimal offboard interface
    * Only uses the python console 
    * Great for debugging and testing


How it works
---------------

* Navigation nodes should be able to : 
    * Arm/Disarm
    * Can implement landing/takeoff methods
    * Send setpoints to `mavros/setpoint_position/local` or any `mavros/setpoint_*/local`. 


Available JSON scenari
-------------

JSON missions are sent though the web application (flyingros\_web) via the task_node

* [scenario_web_circle.json](https://github.com/AlexisTM/flyingros/tree/master/flyingros_nav/scenari/scenari_web/scenario_web_circle.json) - Make a circle
* [scenario_web_square.json](https://github.com/AlexisTM/flyingros/tree/master/flyingros_nav/scenari/scenari_web/scenario_web_square.json) - Make a square