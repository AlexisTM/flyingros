FlyingROS Installation
=============

## Step by step

### Install ROS (Indigo or Kinetic), any mode.

```
http://wiki.ros.org/kinetic/Installation/Ubuntu

or

http://wiki.ros.org/indigo/Installation/Ubuntu
```

### Install project dependencies 


```
sudo apt-get update
sudo apt-get upgrade

# Prebuilt
sudo apt-get install ros-kinetic-mavros-extras ros-kinetic-rosbridge-suite 

# Prepare your catkin workspace
mkdir ~/Workspace
mkdir ~/Workspace/src
cd Workspace/src
catkin_init_workspace

# usb_cam (ros-indigo-usb-cam for indigo, build from source on Kinetic)
~/Workspace/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git

# [Swiftnav Piksi RTK](http://wiki.ros.org/swiftnav_piksi)
cd ~/mysrc       # cd to a directory where you will download and build libsbp
git clone https://github.com/swift-nav/libsbp.git
cd libsbp/c
mkdir build
cd build
cmake ../
make
sudo make install   # install headers and libraries into /usr/local

cd ~/Workspace/src      # your catkin workspace
git clone https://github.com/PaulBouchier/swiftnav_piksi.git

# FlyingROS
cd ~/Workspace/src
git clone https://github.com/AlexisTM/flyingros.git


# make and source the workspace
cd ~/Workspace 
catkin_make
source ~/Workspace/devel/setup.bash


```
