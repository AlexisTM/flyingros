Six Lasers under ROS
===============

This Arduino sketch is used to make the acquisition off all 6 the lasers, using [ROSSerial](https://github.com/ros-drivers/rosserial) and [LidarEnhanced](https://github.com/AlexisTM/LIDAREnhanced/).

To use it, you have to install those libraries. For LidarEnhanced, you can download the .zip and simply use the "Install a .zip library" from the Arduino IDE. For RosSerial, you need to compile Flyingros and source it (as the library send a custom message - `flyingros_msgs/MultiEcho`). Then generate the library via `rosrun rosserial_Arduino make_libraries ~/` and copy the generated library `~/ros_lib` to the Arduino library folder (`Documents/Arduino/Libraries` in Windows). 

