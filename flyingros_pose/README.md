Flyingros_pose
==============

Flyingros_pose is the localization package. You can add new localization method. As this the "main" part of an autonomous multicopter, there is a whole tutorial helping you to choose your localisation(s) method.

Go check the [TUTORIALS](tutorials)

Main idea
--------

* Based on MSF (Multi Sensor Fusion - from ethzasl)
* Merge IMU 
* Merge Sensors
* Output the position to `/mavros/mocap/pose`

Available data
------

* PixHawk IMU
* GPS RTK (as local NED)
* Lasers, altitude, position, yaw angle
* Onboard GPS
* Camera (SLAM)
* Pozyx/Marvelmind

Camera choice
-----

* mvBlueFOX-IGC200wG-1112 (~320 €)
* Vimba on ARMv7 HF did not worked well for us on the Odroid, the frames where incomplete.
* USB customer camera (Odorid 720p for example) : To avoid, too slow and no global shutter.
