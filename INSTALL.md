FlyingROS Installation
=============

## Step by step

### Install ROS (Indigo or Kinetic), any mode.

```
http://wiki.ros.org/kinetic/Installation/Ubuntu

or

http://wiki.ros.org/indigo/Installation/Ubuntu
```

### XU3-lite/XU3/XU4 only

Improve compiled programs performances by adding CFLAGS at the end of your .bashrc

```
export CFLAGS="-march=armv8-a+crc -mtune=cortex-a53 -fexpensive-optimizations -fprefetch-loop-arrays -mfpu=neon-fp-armv8"
```

Compile GCC6.2

```
wget http://gcc.parentingamerica.com/releases/gcc-6.2.0/gcc-6.2.0.tar.bz2
tar xf gcc-6.2.0.tar.bz2
cd gcc-6.2.0
contrib/download_prerequisites
mkdir build && cd build
../configure -v --enable-languages=c,c++ --prefix=/usr/local/gcc-6 --program-suffix=-6 --with-arch=armv8hf-a+crc --with-fpu=vfp --with-float=hard --build=arm-linux-gnueabihf --host=arm-linux-gnueabihf --target=arm-linux-gnueabihf
make -j8
sudo make install
# add to .bashrc
export PATH=/usr/local/gcc-6/bin:$PATH
```

### Install project dependencies

NOTE: You might need some more SWAP if on the Odroid

```
touch /swapfile
sudo chmod 600 /swapfile
sudo fallocate -l 4G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon -s
```


#### Installation

```
sudo apt-get update
sudo apt-get dist-upgrade

# Prebuilt
sudo apt-get install ros-kinetic-mavros-extras ros-kinetic-rosbridge-suite

# Prepare your catkin workspace
mkdir ~/External
mkdir ~/Workspace
mkdir ~/Workspace/src
cd Workspace/src
catkin_init_workspace

# usb_cam (ros-indigo-usb-cam for indigo, build from source on Kinetic)
~/Workspace/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git

# [Swiftnav Piksi RTK](http://wiki.ros.org/swiftnav_piksi)
cd ~/External       # cd to a directory where you will download and build libsbp
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

# If you want URWID interfaces (in flyingros_nav, console_* scripts)
cd ~/External
wget https://pypi.python.org/packages/source/u/urwid/urwid-1.3.1.tar.gz
tar -xvf urwid-1.3.1.tar.gz
rm urwid-1.3.1.tar.gz
cd urwid-1.3.1
sudo python setup.py install
```

### Install SLAM capabilities

Please check the [`flyingros_pose` package](flyingros_pose/INSTALL.md).
