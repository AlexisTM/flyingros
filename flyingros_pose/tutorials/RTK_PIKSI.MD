RTK Piksi
==============

The Piksi RTK GPS was a cheap RTK (500$) which is now 6 time the price (3000$). If you want to go with a cheap RTK, simply use ublox (M8P).

Guide : http://wiki.ros.org/swiftnav_piksi

Dependencies
-------------

```bash
cd mysrc       # cd to a directory where you will download and build libsbp
git clone https://github.com/swift-nav/libsbp.git
cd libsbp/c
mkdir build
cd build
cmake ../
make
sudo make install   # install headers and libraries into /usr/local
```

Driver
------------

NOTE : Don't forget to source the catkin workspace

```bash
cd catkin_ws/src       # your catkin workspace
git clone https://github.com/PaulBouchier/swiftnav_piksi.git
cd ..
catkin_make
``` 

Start 
-------------

```bash
rosrun flyingros_web rtk_csv
rosrun swiftnav_piksi piksi_node 
```