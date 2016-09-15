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
* Lasers
* PixHawk IMU
* Onboard GPS

Future
-------

* Camera (SLAM or equivalent)
