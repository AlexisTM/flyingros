Mavros installation note
====================

Mavros is a needed hardware abstraction layer to link MavLink to ROS (NED <=> ENU and converting messages). Here is the method to install it from source, if you are using **indigo** and you are unable to put in offboard mode (because of mavlink v2 version).

Dependencies 
-----------------

Ros build tools, other than catkin.

```bash
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
$ pip install future
```

Make the workspace
----------------

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ wstool init .
```

Building 
-----------------

Checkout the mavros version you want to use (0.18+ for mavlink2, with the latest firmware)

```bash
$ git clone https://github.com/mavlink/mavros
$ git checkout 0.18.4 
```

Adding MavLink to the catkin repo

```bash
export ROS_DISTRO=kinetic # to be sure you get the v2 version of Mavlink
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall 
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

And build

```
export ROS_DISTRO=indigo 
catkin_make_isolated -j1 # It is important to reduce the number of active cores if you are building it on an Odroid or RPi. Also, you should add some swap (swapon).
```

You now have a working mavros node, which can enable offboard mode.
