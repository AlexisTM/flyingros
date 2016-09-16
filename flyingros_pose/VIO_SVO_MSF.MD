VIO SVO MSF
==============

Installation of SVO and MSF together. SVO makes the Vision Odometry and MSF merge sensors (initially only the IMU and the Vision Odometry).

To make SVO works well 
-----------------------

* SVO only works well with downward-facing cameras.
* Wide camera angle (>90º, but about 110 is feasible)
* High frame rate (reviewers talk about ~70fps)
* Camera with fixed exposure time (ISO), fixed shutter speed, avoid rolling shutter cameras

Installation
---------------

Tested on the Odroid XU4

```
# It activate NEON FLAGS for ARM
echo export ARM_ARCHITECTURE=True > ~/.bashrc 

# apt-get dependencies
sudo apt-get install cmake libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake libqglviewer-dev 

# Create a catkin workspace and external workspace
mkdir ~/Workspace
mkdir ~/Workspace/External
mkdir ~/Workspace/Catkin
mkdir ~/Workspace/Catkin/src
cd ~/Workspace/Catkin/src
catkin_init_workspace

# Installation https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-ROS
# Sophus
cd ~/Workspace/External
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
sudo make install # you can skip it for sophus & fast

# FAST
cd ~/Workspace/External
git clone https://github.com/uzh-rpg/fast.git
cd fast
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
sudo make install # you can skip it for sophus & fast

# G2O
cd ~/Workspace/External
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make -j5
sudo make install

cd ~/Workspace/Catkin/src
git clone https://github.com/uzh-rpg/rpg_svo
git clone https://github.com/ethz-asl/ethzasl_msf
```
