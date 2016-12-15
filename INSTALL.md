Installation
=============

## Step by step

### Install ROS (Indigo or Kinetic), any mode.

```
http://wiki.ros.org/kinetic/Installation/Ubuntu

or

http://wiki.ros.org/indigo/Installation/Ubuntu
```

#### Installation

```
sudo apt-get update
sudo apt-get dist-upgrade

# Prebuilt
sudo apt-get install ros-kinetic-mavros-* ros-kinetic-rosbridge-suite

# Prepare your catkin workspace
mkdir ~/External
mkdir ~/Workspace
mkdir ~/Workspace/src
cd Workspace/src
catkin_init_workspace

# FlyingROS
cd ~/Workspace/src
git clone https://github.com/AlexisTM/flyingros.git

# make and source the workspace
cd ~/Workspace
catkin_make
source ~/Workspace/devel/setup.bash

# If you want URWID interfaces (in flyingros_nav, console_* scripts)
cd ~/External
wget https://pypi.python.org/packages/source/u/urwid/urwid-1.3.1.tar.gz
tar -xvf urwid-1.3.1.tar.gz
rm urwid-1.3.1.tar.gz
cd urwid-1.3.1
sudo python setup.py install
```
