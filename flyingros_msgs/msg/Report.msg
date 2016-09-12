# This message is a complete report for the UAV to be sent via TCP/UDP
# Sending a whole message instead of 15 messages reduce the network
# overhaul/number of connections
std_msgs/Header header
RPYPose mocap
RPYPose local
RPYPose setpoint
Distance lasers_raw
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 linear_speed
geometry_msgs/Vector3 angular_speed
sensor_msgs/NavSatFix gps_precision
nav_msgs/Odometry gps_odometry
Battery battery
bool connected
bool armed
bool guided
string mode
