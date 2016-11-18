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

Additionnal informations 
----------------------

* [MSF.MD](MSF.MD) - Generic informations
* [IMPLEMENT_A_NODE.MD](MSF.MD) - Implement your node (simplest way)
