FlyingROS_pose installation
===========================

This tutorial helps you to install SLAM dependencies.

First, you have to create a catkin workspace : 

```
cd 
mkdir Workspace
mkdir Workspace/Catkin
mkdir Workspace/Catkin/src
cd Workspace/Catkin/src
catkin_init_workspace
```

ethz-asl ROVIO
----------------

https://github.com/ethz-asl/rovio/wiki

### Dependencies

### Installation

```
cd ~/Workspace/Catkin/src
git clone https://github.com/ethz-asl/rovio
git clone https://github.com/ethz-asl/kindr
git clone https://github.com/ethz-asl/glog_catkin
git clone https://github.com/ethz-asl/catkin_simple
git clone https://github.com/ethz-asl/kalibr
cd rovio
git submodule update --init --recursive
cd ..
catkin_make
```

ethz-asl PTAM
----------------

### Dependencies


### Installation

```
cd ~/Workspace/Catkin/src
git clone https://github.com/ethz-asl/ethzasl_apriltag
git clone https://github.com/ethz-asl/ethzasl_ptam
git clone https://github.com/ethz-asl/ethzasl_sensor_fusion
# or git clone https://github.com/ethz-asl/ethzasl_msf

cd ..
catkin_make
```

JuanTarrio/rebvo
-----------------

https://github.com/JuanTarrio/rebvo

NOT FINISHED

### Dependencies

```
# NE10 - for optimisations
cd ~/Workspace/External 
git clone https://github.com/projectNe10/Ne10
mkdir Ne10/build
cd Ne10/build
cmake .. -DNE10_LINUX_TARGET_ARCH=armv7 -DGNULINUX_PLATFORM -DCMAKE_BUILD_TYPE=Release

# TooN
cd ~/Workspace/External 
git clone https://github.com/edrosten/TooN
cd TooN 
./configure && make && sudo make install
```