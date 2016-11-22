Drone localisation
==================

This tutorial section focuse on all the existing localisation systems with a tutorial for each system. 

It will helps you to choose yours and finally fuse them through MSF with the IMU (Multi Sensor Fusion).

Pose Sensor (6DOF, 4DOF)
===========

* [Fixed lasers (at least 6 lasers):](LASERS.MD)
  * Price: 750$
  * Precision:  +/- 5cm
  * Range: 25m
  * Rate: 100Hz
  * 4 DOF
* 3D Rotating laser (Velodyne):
  * Price: 7500$
  * Precision:  +/- 2cm
  * Range: 300m
  * Rate : 20M points/s
  * 3,4 DOF
* [Camera - Computer vision algorithms:](VISION_ODOMETRY/README.MD)
  * Price: 350$
  * Precision:  +/- 2cm
  * Range: 300m
  * Rate: 100Hz
  * 3,4,6 DOF
* VICON and MOCAP:
  * Price: 60k$ to 120k$
  * Precision:  +/- 2cm
  * Range: 10x10x10m
  * Rate: 20Hz
  * 6 DOF


Position Sensors (3DOF)
===============

* GPS:
  * Price: 20$ to 60$
  * Precision:  +/- 5m
  * Range: Earth
  * Rate: 1Hz
* D-GPS:
  * Price: 20$ to 60$
  * Precision:  +/- 20cm
  * Range: Earth with stationnary satellite available
  * Rate: 1Hz
  * Additionnal: Random drift, bad repetability
* [RTK](RTK_PIKSI.MD):
  * Price: 300$ (ublox), 2k$ (Piksi) to 5k$ (others)
  * Precision:  +/- 2cm when fixed
  * Range: Earth on clear space (30° of the antenna)
  * Rate: 25Hz
  * Additionnal: Have to wait the "fix" RTK, needs a base station (pretty small)
  * 4DOF with 2 receptors
* UWB (Ultrawide band, Decawave, Pozyx.io)
  * Price: 500$
  * Precision:  +/- 10cm
  * Range: 100m
  * Rate: 20Hz
  * 4DOF with 2 receptors
* Ultrasound (marvelmind) 
  * Price: 365$
  * Precision:  +/- 10cm
  * Range: 50m
  * Rate: 20Hz
  * 4DOF with 2 receptors

Altitude Sensors (1DOF)
==============

* Lasers:
  * Price: 150$
  * Precision:  +/- 2cm
  * Range: 25m
  * Rate: 500Hz
* Ultrasonic:
  * Price: 2$
  * Precision:  +/- 2cm
  * Range: 2m
  * Rate: 20Hz
* Pressure sensor:
  * Price: 5$
  * Precision:  +/- 20 cm (need more info)
  * Range: Up to the troposphere, maybe more

