FlyingROS_msgs
==============

FlyingROS_msgs is the messaging package. You can add new messages to use in other packages.

Available messages
--------

* [Report.msg](msg/Report.msg) - Report the drone status
* [Mission.msg](msg/Mission.msg) - Mission, multiple tasks
* [Task.msg](msg/Task.msg) - Task to do by the UAV
* [Battery.msg](msg/Battery.msg) - Battery status
* [RPYPose.msg](msg/RPYPose.msg) - Roll Pitch Yaw Position 
* [Distance.msg](msg/Distance.msg) - Raw laser messages (deprecated, replaced by MultiEcho)
* [MultiEcho.msg](msg/MultiEcho.msg) - Laser measures and strength
* [RPY.msg](msg/RPY.msg) - Roll Pitch Yaw
* [PositionWithCovariance.msg](msg/PositionWithCovariance.msg) - Position with covariance (like PoseWithCovariance, without orientation)
* [PositionWithCovarianceStamped.msg](srv/PositionWithCovarianceStamped.msg) - Position with covariance and stamp (like PoseWithCovarianceStamped, without orientation)

Available services
----------

* [MissionHandle.srv](srv/MissionHandle.srv) - Ask to handle a mission (send a mission)
* [MissionRequest.srv](srv/MissionRequest.srv) - Ask to give back a mission (get a mission)
* [TaskHandle.srv](srv/TaskHandle.srv) - Ask to handle a task (send a task)
* [TaskRequest.srv](srv/TaskRequest.srv) - Ask to give back a task (get a task) 