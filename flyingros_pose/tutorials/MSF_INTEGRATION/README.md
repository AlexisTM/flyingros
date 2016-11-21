MSF_INTEGRATION
================

MSF is a **modular** framework used to fuse data from multiple sensors. It actulally can merge the IMU, GPS, Altitude, Position, Pose (position + orientation) and a spherical position.

You can check [MSF.MD](MSF.MD) for more detailed *informations* about the framework and the *installation*.

Sensors implemented `ethzasl_msf\msf_updates\include`
-----------------

* msf_pose_pressure_sensor : altitude 
  * &lt;geometry_msgs::PointStamped&gt; to `msf_updates/topic_namespace/pressure_height`
* msf_position_sensor : position
  * &lt;geometry_msgs::PointStamped&gt; to `msf_updates/topic_namespace/position_input`
  * &lt;geometry_msgs::TransformStamped&gt; to `msf_updates/topic_namespace/transform_input`
  * &lt;sensor_msgs::NavSatFix&gt; to `msf_updates/topic_namespace/navsatfix_input`
* msf_pose_sensor : position and orientation
  * &lt;geometry_msgs::PoseWithCovarianceStamped&gt; to `msf_updates/topic_namespace/pose_with_covariance_input`
  * &lt;geometry_msgs::TransformStamped&gt; to `msf_updates/topic_namespace/transform_input`
  * &lt;geometry_msgs::PoseStamped&gt; to `msf_updates/topic_namespace/pose_input`
* msf_spherical_position : position from external pointing (2 angle as roll is useless) and a distance
  * &lt;geometry_msgs::PointStamped&gt; to `msf_updates/topic_namespace/angle_input`

Nodes implementing sensors `ethzasl_msf\msf_updates\src`
-----------------------

* pose_msf
  * IMU
  * msf_pose_sensor
* pose_pressure_msf
  * IMU
  * msf_pose_sensor
  * msf_pressure_sensor
* position_msf
  * IMU
  * msf_position_sensor
* position_pose_msf
  * IMU
  * msf_pose_sensor
  * msf_position_sensor
* spherical_msf
  * IMU
  * msf_spherical_position (distance + 2 angle from an extern fixed point, pointing the copter.)
* pose_position_pressure_msf (available in [AlexisTM/ethzasl_msf](https://github.com/AlexisTM/ethzasl_msf/))
  * IMU 
  * msf_pose_sensor (vision)
  * msf_position_sensor (RTK position or GPS)
  * msf_pressure_sensor (Altitude from lasers)
* dual_position_msf (available in [AlexisTM/ethzasl_msf](https://github.com/AlexisTM/ethzasl_msf/))
  * IMU 
  * msf_position_sensor (RTK position or GPS)
  * msf_position_sensor (RTK position or GPS)
* position_pressure_msf (available in [AlexisTM/ethzasl_msf](https://github.com/AlexisTM/ethzasl_msf/))
  * IMU 
  * msf_position_sensor (RTK position or GPS)
  * msf_pressure_sensor (Altitude from lasers)

Starting MSF
---------------

For example; for the RTK + Pixhawk integration, with this launch file 

```xml
<launch>
    <!-- Pixhawk config -->
    <arg name="fcu_url" default="serial:///dev/ttySAC0:921600" />
    <arg name="gcs_url" default="udp://:14556@127.0.0.1:14550" />
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="50" />
    <arg name="log_output" default="screen" />
    <include file="$(find mavros)/launch/node.launch">
      <arg name="pluginlists_yaml" value="$(find flyingros_pose)/cfg/pixhawk/px4_pluginlists.yaml" />
      <arg name="config_yaml" value="$(find flyingros_pose)/cfg/pixhawk/px4_config.yaml" />
      <arg name="fcu_url" value="$(arg fcu_url)" />
      <arg name="gcs_url" value="$(arg gcs_url)" />
      <arg name="tgt_system" value="$(arg tgt_system)" />
      <arg name="tgt_component" value="$(arg tgt_component)" />
      <arg name="log_output" value="$(arg log_output)" />
    </include>
    <node name="rtk" pkg="flyingros_pose" type="rtk" clear_params="true" output="screen">
    </node>
    <node pkg="swiftnav_piksi" type="piksi_node" name="piksi_node" output="screen">
     <param name="port" value="/dev/ttyUSB0" />
   </node>
   <!-- rosbag record /mavros/imu/data  /flyingros/rtk/position /gps/rtkfix /gps/fix /msf_core/pose_after_update  /msf_core/pose /msf_core/odometry /mavros/local_position/pose /mavros/mocap/pose msf_core/pose -->
   <!-- MSF config -->
   <!-- rosrun dynamic_reconfigure dynparam set /position/position_sensor core_init_filter true -->
    <node name="position" pkg="msf_updates" type="position_sensor" clear_params="true" output="screen">
        <remap from="msf_core/imu_state_input" to="/mavros/imu/data"/>
        <remap from="msf_updates/position_input" to="/flyingros/rtk/position" />
        <rosparam file="$(find flyingros_pose)/cfg/msf/px4_fix.yaml"/>
        <rosparam file="$(find flyingros_pose)/cfg/msf/position_sensor.yaml"/>
    </node>
</launch>
```

You need to launch the launchfile then init the filter.

```bash
roslaunch flyingros msf_rtk_only.launch
rosrun dynamic_reconfigure dynparam set position/position_sensor core_init_filter true
```

Note the dynamic\_reconfigure command can be called from rqt\_reconfigure instead. For other launch files, you can simply change the command to 

```bash
rosrun dynamic_reconfigure dynparam set node_name/type_name param data
```

Additionnal informations 
----------------------

* [MSF.MD](MSF.MD) - Generic informations
* [IMPLEMENT_A_NODE.MD](MSF.MD) - Implement your node (simplest way)
