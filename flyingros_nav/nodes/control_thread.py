#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
send_setpoint.py

This script sends positions to control the UAV in X, Y, Z

ILPS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ILPS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with ILPS.  If not, see <http://www.gnu.org/licenses/>.

Software created by Alexis Paques and Nabil Nehri for the UCL
in a Drone-Based Additive Manufacturing of Architectural Structures
project financed by the MIT Seed Fund

Originaly published by Vladimir Ermakov (c) 2015 under GNU GPLv3
Copyright (c) Alexis Paques 2016
Copyright (c) Nabil Nehri 2016
"""

from __future__ import division
import rospy
import mavros
import time
import tf
import numpy as np
from getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Range
from mavros.utils import *
from algorithm_functions import rad2degf, deg2radf
from transformations import *
from laserpack.msg import Distance, Task

# Callbacks
def Task_Callback(data):
    # Output data
    global setpoint, pose
    # Classes
    global arming_client, set_mode_client

    # GOTO
    if data.mission_type == 0 :
        setpoint.x = data.position.x
        setpoint.y = data.position.y
        setpoint.z = data.position.z

    # Set offboard
    if data.mission_type == 9 :
        set_mode_client(custom_mode = "OFFBOARD")

    # Disarm
    if data.mission_type == 11 :
        set_mode_client(custom_mode = "OFFBOARD")
        arming_client(False)

    # Arm
    if data.mission_type == 13 :
        setpoint.z = 0
        setpoint.x = pose.pose.position.x
        setpoint.y = pose.pose.position.y
        set_mode_client(custom_mode = "OFFBOARD")
        arming_client(True)

    # Landing
    if data.mission_type == 123:
        setpoint.z = 1
        while pose.pose.position.z > 1.05 :
            time.sleep(0.1)
        time.sleep(0.5)

        setpoint.z = 0.65
        while pose.pose.position.z > 0.75 :
            time.sleep(0.1)
        time.sleep(0.5)

        setpoint.z = 0.1
        while pose.pose.position.z > 0.35 :
            time.sleep(0.1)

        setpoint.z = 0
        while pose.pose.position.z > 0.25 :
            time.sleep(0.1)
        arming_client(False)

    # Takeoff
    if data.mission_type == 122:
        set_mode_client(custom_mode = "OFFBOARD")
        arming_client(True)
        setpoint.z = 0
        setpoint.x = pose.pose.position.x
        setpoint.y = pose.pose.position.y
        time.sleep(1)

        setpoint.z = 1.5
        while pose.pose.position.z < 1.0 :
            time.sleep(0.1)
        time.sleep(0.5)

        setpoint.z = 1.25
        time.sleep(0.5)
        while pose.pose.position.z < 1.0 or pose.pose.position.z > 1.35:
            time.sleep(0.1)

        setpoint.z = 1
        time.sleep(0.5)
        while pose.pose.position.z < 0.9 or pose.pose.position.z > 1.05:
            time.sleep(0.1)

        rospy.loginfo("TAKOFF SUCCESS")
        time.sleep(0.5)

def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def laser_callback(data):
    # Input data
    global laserposition
    # Publishers
    global local_pos_pub
    # Output data
    global laser_position_count

    data.header.stamp=rospy.Time.now()
    data.header.seq=laser_position_count
    # msg = PoseStamped()
    # msg.header.stamp=rospy.Time.now()
    # msg.header.seq=laser_position_count
    # msg.pose.position.x = laserposition.pose.position.x
    # msg.pose.position.y = laserposition.pose.position.y
    # msg.pose.position.z = laserposition.pose.position.z
    # msg.pose.orientation.x = laserposition.pose.orientation.x
    # msg.pose.orientation.y = laserposition.pose.orientation.y
    # msg.pose.orientation.z = laserposition.pose.orientation.z
    # msg.pose.orientation.w = laserposition.pose.orientation.w
    local_pos_pub.publish(data)
    laser_position_count = laser_position_count + 1

def lasers_raw_callback(data):
    global lasers_raw
    lasers_raw = data

def sendLidar():
    global lasers_raw
    lidar_publisher = rospy.Publisher('mavros/distance_sensor/lidar', Range, queue_size=1)
    rate = rospy.Rate(30)
    msg = Range()
    sendLidar_count = 0
    msg.radiation_type = msg.INFRARED
    msg.field_of_view = 0.0523599 # 3° in radians
    msg.min_range = 0.05
    msg.max_range = 20.0
    while run:
        msg.header.stamp = rospy.Time.now()
        msg.header.seq=sendLidar_count
        msg.range = (lasers_raw.lasers[4] + lasers_raw.lasers[5])/200
        lidar_publisher.publish(msg)
        sendLidar_count = sendLidar_count + 1
        rate.sleep()

def sendSetpoint():
    # Input data
    global setpoint, yawSetPoint, run, position_control
    # Output data
    global setPointsCount

    setPointsCount = 0
    #local_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

    rate = rospy.Rate(30)

    while run:
        q = quaternion_from_euler(0, 0, deg2radf(yawSetPoint), axes="sxyz")

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq=setPointsCount
        msg.pose.position.x = float(setpoint.x)
        msg.pose.position.y = float(setpoint.y)
        msg.pose.position.z = float(setpoint.z)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()

        # msg = PositionTarget()
        # if(position_control):
        #     #msg.type_mask = msg.IGNORE_PX or msg.IGNORE_PY or msg.IGNORE_VX or msg.IGNORE_VY or msg.IGNORE_VZ or msg.IGNORE.AFX or msg.IGNORE.AFY or msg.IGNORE.AFZ
        #     msg.type_mask = 2552 #0b1001 1111 1000
        # else :
        #    #  msg.type_mask = msg.IGNORE_VX or msg.IGNORE_VY or msg.IGNORE_VZ or msg.IGNORE.AFX or msg.IGNORE.AFY or msg.IGNORE.AFZ
        #     msg.type_mask = 2555 # 0b1001 1111 1011

        # msg.coordinate_frame = msg.FRAME_LOCAL_NED
        # msg.position.x = float(setpoint.x)
        # msg.position.y = float(setpoint.y)
        # msg.position.z = float(setpoint.z)
        # msg.yaw = 0.0
        # msg.yaw_rate = 1
        # local_setpoint_pub.publish(msg)
        # setPointsCount = setPointsCount + 1
        # rate.sleep()

# If we want to reduce rate of position sent
# def sendPosition():
#     global laserposition
#     global run
#     global laser_position_count
#     global local_pos_pub
#     global position_control
#     rate = rospy.Rate(100)
#     while run:

#         laserposition.header.stamp=rospy.Time.now()
#         laserposition.header.seq=laser_position_count
#         laserposition.pose.position.x = 0.5
#         laserposition.pose.position.y = 1
#         laserposition.pose.position.z = 2.3
#         laserposition.pose.orientation.x = 0
#         laserposition.pose.orientation.y = 0
#         laserposition.pose.orientation.z = 0
#         laserposition.pose.orientation.w = 1

#         local_pos_pub.publish(laserposition)
#         laser_position_count = laser_position_count + 1
#         rate.sleep()

def InterfaceKeyboard():
    # Input data
    global pose, laser_position_count
    # Output data
    global setpoint, yawSetPoint, run, position_control
    # Publishers
    global arming_client, set_mode_client

    what = getch()
    if what == "t":
        setpoint.x = setpoint.x - 0.1
    if what == "g":
        setpoint.x = setpoint.x + 0.1
    if what == "f":
        setpoint.y = setpoint.y - 0.1
    if what == "h":
        setpoint.y = setpoint.y + 0.1
    if what == "i":
        setpoint.z = setpoint.z + 0.1
    if what == "k":
        setpoint.z = setpoint.z - 0.1
    if what == "b":
        yawSetPoint = yawSetPoint + 1
    if what == "n":
        yawSetPoint = yawSetPoint - 1
    if what == "c":
        setpoint.x = pose.pose.position.x
        setpoint.y = pose.pose.position.y
        setpoint.z = pose.pose.position.z

    if what == "p":
        setpoint.x = pose.pose.position.x
        setpoint.y = pose.pose.position.y
        position_control = True
    if what == "l":
        position_control = False
    if what == "q":
        arming_client(False)
    if what == "a":
        arming_client(True)
    if what == "e":
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "m":
        run = False
        time.sleep(1)
        exit()

    Q = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)

    rospy.loginfo("MODE POSITION: ")
    rospy.loginfo("true" if position_control else "false")
    rospy.loginfo("Positions sent : %i",  )
    rospy.loginfo("Position x: %s y: %s z: %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", setpoint.x, setpoint.y, setpoint.z)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", rad2degf(euler[0]))
    rospy.loginfo("pitch : %s", rad2degf(euler[1]))
    rospy.loginfo("yaw : %s", rad2degf(euler[2]))
    rospy.loginfo("wanted yaw : %s", yawSetPoint)

def init():
    # Input data
    # Output data
    global state, setpoint, yawSetPoint, setPointsCount, PositionsCount, \
           run, laser_position_count, laserposition, pose, lasers_raw, position_control
    # Publishers
    global local_pos_pub, arming_client, set_mode_client
    # Objects

    # Global variable initialisation
    lasers_raw = Distance(lasers=[0,0,0,0,0,0], status=[0,0,0,0,0,0])
    pose = PoseStamped()
    laserposition = PoseStamped()
    setpoint = Point()
    setpoint.x = 1
    setpoint.y = 1
    setpoint.z = 1
    # When true, setpoints are positions
    # When false, setpoints is a velocity
    position_control = True
    yawSetPoint = 0
    laser_position_count = 0
    run = True
    setPointsCount = 0
    PositionsCount = 0
    state = State()

    # Node initiation
    rospy.init_node('laserpack_control')

    local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)

    time.sleep(1)

    # Publishers, subscribers and services
    pose_sub        = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    laser_pose_sub  = rospy.Subscriber('lasers/filtered', PoseStamped, laser_callback)
    state_sub       = rospy.Subscriber('mavrqos/state', State, State_Callback)
    laser_sub       = rospy.Subscriber('lasers/raw', Distance, lasers_raw_callback)
    task_sub        = rospy.Subscriber('web/task', Task, Task_Callback)


    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    # Thread to send setpoints
    tSetPoints = Thread(target=sendSetpoint).start()
    # Thread to send Lidar height
    tLidarZ = Thread(target=sendLidar).start()
    # In case we want to send positions with reduced rate, just use this thread
    # tPositions = Thread(target=sendPosition).start()

    while not rospy.is_shutdown():
        InterfaceKeyboard()

if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
