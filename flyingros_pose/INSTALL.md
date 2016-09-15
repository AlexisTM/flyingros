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

### Dependencies

```
sudo add-apt-repository ppa:ethz-asl/common
sudo apt-get update
sudo apt-get install ros-indigo-kindr-*
```

### Installation

```
cd ~/Workspace/Catkin/src
git clone https://github.com/ethz-asl/rovio
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
git clone https://github.com/ethz-asl/ethzasl_msf
git clone https://github.com/ethz-asl/ethzasl_sensor_fusion

cd ..
catkin_make
```
