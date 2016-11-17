MSF_INTEGRATION
================

MSF is a **modular** framework used to fuse data from multiple sensors. It actulally can merge the IMU, GPS, Altitude, Position, Pose (position + orientation) and a spherical position.

You can check [MSF.MD](MSF.MD) for more detailed *informations* about the framework and the *installation*.

Sensors implemented `ethzasl_msf\msf_updates\include`
-----------------

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

Nodes implementing sensors `ethzasl_msf\msf_updates\src`
-----------------------

* pose_msf
  * IMU
  * msf_pose_sensor
* pose_pressure_msf
  * IMU
  * msf_pose_sensor
  * msf_pose_pressure_sensor
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
* lasers_rtk_vision_msf (available in [AlexisTM/ethzasl_msf](https://github.com/AlexisTM/ethzasl_msf/))
  * IMU 
  * msf_pose_sensor (vision)
  * msf_position_sensor (RTK position or GPS)
  * msf_pose_pressure_sensor (altitude from lasers)

Additionnal informations 
----------------------

* [MSF.MD](MSF.MD) - Generic informations
* [IMPLEMENT_A_NODE.MD](MSF.MD) - Implement your node (simplest way)
