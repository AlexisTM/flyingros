#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
control_thread.py
 
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

Originaly inspired of Vladimir Ermakov work (c) 2015 under GNU GPLv3
Copyright (c) Alexis Paques 2016
Copyright (c) Nabil Nehri 2016
"""

from __future__ import division
import rospy
import mavros
import time
import tf
import numpy as np
from threading import Thread
from mavros.utils import *
from geometry_msgs.msg import PoseStamped, Point, Pose
from sensor_msgs.msg import Imu, Range
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from math import pi

# Returns a radian from a degree
def deg2radf(a):
    return float(a)*pi/180

# Returns a degree from a radian
def rad2degf(a):
    return float(a)*180/pi

class _GetchUnix:
    """Fetch and character using the termios module."""
    def __init__(self):
        import tty, sys
        from select import select

    def __call__(self):
        import sys, tty, termios
        from select import select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            [i, o, e] = select([sys.stdin.fileno()], [], [], 1)
            if i:
                ch = sys.stdin.read(1)
            else:
                ch = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

getch = _GetchUnix()


def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def sendSetpoint():
    # Input data
    global setpoint, yawSetPoint, run, position_control
    # Output data
    local_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=0)

    rate = rospy.Rate(5)

    while run:
        q = tf.transformations.quaternion_from_euler(0, 0, deg2radf(yawSetPoint), axes="sxyz")

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(setpoint.x)
        msg.pose.position.y = float(setpoint.y)
        msg.pose.position.z = float(setpoint.z)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        local_setpoint_pub.publish(msg)
        rate.sleep()

def InterfaceKeyboard():
    # Input data
    global pose
    # Output data
    global setpoint, yawSetPoint, run, position_control
    # Publishers
    global arming_client, set_mode_client, lasers_yaw

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
        yawSetPoint = yawSetPoint + 45
    if what == "n":
        yawSetPoint = yawSetPoint - 45
    if what == "c":
        setpoint.x = pose.pose.position.x
        setpoint.y = pose.pose.position.y
        setpoint.z = pose.pose.position.z
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

    rospy.loginfo("Position x: %s y: %s z: %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", setpoint.x, setpoint.y, setpoint.z)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", rad2degf(euler[0]))
    rospy.loginfo("pitch : %s", rad2degf(euler[1]))
    rospy.loginfo("yaw : %s and from lasers %s", rad2degf(euler[2]), rad2degf(lasers_yaw))
    rospy.loginfo("wanted yaw : %s", yawSetPoint)

def init():
    # Input data
    # Output data
    global state, setpoint, yawSetPoint, \
           run, laserposition, pose, lasers_raw, position_control
    # Publishers
    global local_pos_pub, arming_client, set_mode_client, lasers_yaw
    # Objects

    lasers_yaw = 0

    # Global variable initialisation
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
    run = True
    state = State()

    # Node initiation
    rospy.init_node('control_position_setpoint_py')

    time.sleep(1)

    # Publishers, subscribers and services
    pose_sub        = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, Pose_Callback)
    state_sub       = rospy.Subscriber('/mavros/state', State, State_Callback)

    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    # Thread to send setpoints
    tSetPoints = Thread(target=sendSetpoint).start()

    while not rospy.is_shutdown():
        InterfaceKeyboard()

if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
