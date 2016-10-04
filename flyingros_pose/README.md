FlyingROS_pose
==============

FlyingROS_pose is the localization package. You can add new localization method.

Available localization methods
--------

* Six lasers, two in each direction, for rectangular fly space
* GPS RTK
* GPS RTK Fused with lasers for altitude
* SLAM (rovio or PTAM), check [INSTALL notes](INSTALL.md)

Available data
------

* GPS RTK (as local NED)
* Lasers, altitude or position
* PixHawk IMU
* Onboard GPS
* Camera (to be selected)

Camera choice
-----

*
* Vimba on ARMv7 HF did not worked well for us on the Odroid, the frames where incomplete.
* USB customer camera (Odorid 720p for example) : To avoid, too slow and no global shutter.

TODO
------

* Find the right camera
* Make a successfull use of `rpg_svo in` coordination with `ethzasl_msf`.

Future
-------

* Camera (SLAM or equivalent)
