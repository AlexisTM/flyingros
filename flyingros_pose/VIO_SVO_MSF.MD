VIO SVO MSF - NOT RUNNED YET 
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

# G2O - used to run Bundle Adjustment, it increase (x10) the precision at cost of computationnal power
# If you have problems to compile, use : sudo apt-get install ros-indigo-libg2o
cd ~/Workspace/External
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make -j5
sudo make install

cd ~/Workspace/Catkin/src
git clone https://github.com/uzh-rpg/rpg_svo # Vision Odometry
git clone https://github.com/ethz-asl/ethzasl_msf # Sensor vision
git clone https://github.com/uzh-rpg/rpg_vikit.git # SVO tools
git clone https://github.com/ethz-asl/ethzasl_ptam # PTAM calibrator for ATAN calibration

# If you installed G2O, set HAVE_G2O in svo/CMakeLists.txt to true
# also export export G2O_ROOT=$HOME/installdir
# if you installed it via apt-get, the install dir is G2O_ROOT=/opt/ros/indigo/include/g2o/

cd ~/Workspace/Catkin/
catkin_make
# NOTE: I needed at least 4Go RAM to compile => add some swap

echo "source ~/Workspace/Catkin/devel/setup.bash" > ~/.bashrc
```

Camera calibration
-----------------

Choice of calibration - https://github.com/uzh-rpg/rpg_svo/wiki/Camera-Calibration

ATAN calibration (from ethzasl_ptam package) - http://wiki.ros.org/ethzasl_ptam/Tutorials/camera_calibration

Pinhole calibration (ROS & OpenCV standard) - http://wiki.ros.org/camera_calibration

Known issues with installation
---------------------------

If you want G2O to be used and got G2O_INCLUDE_DIR=NOT_FOUND, then you can call catkin_make -DG2O_INCLUDE_DIR=/opt/ros/indigo/include/g2o/
