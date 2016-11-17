[MSF-ETHZASL](https://github.com/ethz-asl/ethzasl_msf)
================

MSF - Modular framework for multi sensor fusion based on an Extended Kalman Filter (EKF)

Dependencies
-------------

Installation 
-------------

```bash
cd ~/Workspace/Catkin/src
git clone https://github.com/ethz-asl/ethzasl_msf
cd ~/Workspace/Catkin
catkin_make -j1 # On the Odroid, don't build with too many cores or it will probably freeze and you will have to reboot.
```

Sensor type 
------------

* msf_pose_pressure_sensor : altitude
  * <geometry_msgs::PointStamped> to `msf_updates/topic_namespace/pressure_height`
* msf_position_sensor : position
  * <geometry_msgs::PointStamped> to `msf_updates/topic_namespace/position_input`
  * <geometry_msgs::TransformStamped> to `msf_updates/topic_namespace/transform_input`
  * <sensor_msgs::NavSatFix> to `msf_updates/topic_namespace/navsatfix_input`
* msf_pose_sensor : position and orientation
  * <geometry_msgs::PoseWithCovarianceStamped> to `msf_updates/topic_namespace/pose_with_covariance_input`
  * <geometry_msgs::TransformStamped> to `msf_updates/topic_namespace/transform_input`
  * <geometry_msgs::PoseStamped> to `msf_updates/topic_namespace/pose_input`
* msf_spherical_position : position from external pointing (2 angle as roll is useless) and a distance
  * <geometry_msgs::PointStamped> to `msf_updates/topic_namespace/angle_input`

Configurations 
-----------

### Pixhawk configuration 

from [google groups](https://groups.google.com/forum/#!topic/px4users/Nv5nZ8PrsKM)

```yaml
scale_init: 1
data_playback: false

#########IMU PARAMETERS#######
####### pixhawk - MPU6050
core/core_noise_acc: 0.003924    # [m/s^2/sqrt(Hz)] mpu6000 datasheet
core/core_noise_gyr: 0.00008726  # [rad/s/sqrt(Hz)] mpu6000 datasheet
core/core_fixed_bias: true
core/core_noise_gyrbias: 0.0     # For fixed bias we do not need process noise.
core/core_noise_accbias: 0.0     # For fixed bias we do not need process noise.


####### Pose sensor
pose_sensor/pose_fixed_scale: false
pose_sensor/pose_noise_scale: 0.0
pose_sensor/pose_noise_p_wv: 0.0
pose_sensor/pose_noise_q_wv: 0.0
pose_sensor/pose_noise_q_ic: 0.0
pose_sensor/pose_noise_p_ic: 0.0
pose_sensor/pose_delay: 0.02
pose_sensor/pose_noise_meas_p: 0.005
pose_sensor/pose_noise_meas_q: 0.02
pose_sensor/pose_initial_scale: 1

# q_ic is the quaternion representing the rotation of the camera in IMU frame. Unit quaternion here as we rotate the coordinate frames in SVO parameters.
pose_sensor/init/q_ic/w: 1.0
pose_sensor/init/q_ic/x: 0.0
pose_sensor/init/q_ic/y: 0.0
pose_sensor/init/q_ic/z: 0.0

# p_ic is the translation between the IMU and the camera in meters.
pose_sensor/init/p_ic/x: 0.0  
pose_sensor/init/p_ic/y: 0.0
pose_sensor/init/p_ic/z: 0.0

pose_sensor/pose_absolute_measurements: true
pose_sensor/pose_use_fixed_covariance: true
pose_sensor/pose_measurement_world_sensor: false # we do not publish the world in camera frame as set in SVO parameters.

pose_sensor/pose_fixed_scale: false
pose_sensor/pose_fixed_p_ic: true
pose_sensor/pose_fixed_q_ic: true
pose_sensor/pose_fixed_p_wv: false
pose_sensor/pose_fixed_q_wv: false


####### Position sensor
position_pose_sensor/pose_fixed_scale: false
position_pose_sensor/pose_fixed_p_ic: false
position_pose_sensor/pose_fixed_q_ic: false
position_pose_sensor/pose_fixed_p_wv: false
position_pose_sensor/pose_fixed_q_wv: false

position_sensor/position_absolute_measurements: true
position_sensor/position_use_fixed_covariance: true
position_sensor/position_measurement_world_sensor: true

#init position offset prism imu
position_sensor/init/p_ip/x: 0.00
position_sensor/init/p_ip/y: 0.00
position_sensor/init/p_ip/z: 0.00

position_sensor/position_absolute_measurements: true
position_sensor/position_use_fixed_covariance: true
position_sensor/position_measurement_world_sensor: true  # selects if sensor measures its position w.r.t. world (true, e.g. Vicon) or the position of the world coordinate system w.r.t. the sensor (false, e.g. ethzasl_ptam)

####### Position presure sensor (altitude)
```` 

launch file 

```
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
    <!-- MSF config -->
    <node name="msf_position_filter" pkg="msf_updates" type="position_sensor" clear_params="true" output="screen">
        <remap from="msf_core/imu_state_input" to="/mavros/imu/data">
        <remap from="msf_updates/position_input" to="/flyingros/rtk/position" />
        <remap from="msf_updates/navsatfix_input" to="/mavros/global_position/global" />
        <remap from="msf_updates/pressure_height" to="/flyingros/lasers/altitude" />
        <rosparam file="$(find flyingros_pose)/cfg/msf/px4_fix.yaml"/>
        <rosparam file="$(find flyingros_pose)/cfg/msf/pose_sensor.yaml"/>
        <rosparam file="$(find flyingros_pose)/cfg/msf/position_sensor.yaml"/>
    </node>

</launch>
```