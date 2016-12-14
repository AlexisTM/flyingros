Flyingros_libs
==============

Flyingros_libs is the library package. You can add new libraries to use them somewhere else.

Available libraries
--------

* transformations.py - Quaternion transformations in python 
* tasks.py - Tasks to use the UAV (UAV representation, Controller and tasks)
* getch.py - Gives a simple way to add a user interface with the keyboard
* algotithm_functions.py - helper functions used in the localization algotithms 

DEPRECATED
---------

* kalman.py - replaced by MSF (ethasl), Multiple implementation of diverse filters
* lasers.py - replaced by `flyingros_pose/cfg/laser/*laser.yaml` old way (pre-ROS) to carry data on Python algorithm
