#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
console_task.py

This script is made to use the task controller through a console gui.

This file is part of FlyingROS (Indoor Laser Positioning System).

FlyingROS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

FlyingROS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with FlyingROS.  If not, see <http://www.gnu.org/licenses/>.

Software created by Alexis Paques and Nabil Nehri for the UCL
in a Drone-Based Additive Manufacturing of Architectural Structures
project financed by the MIT Seed Fund

Copyright (c) Alexis Paques 2016
"""

from __future__ import division
import urwid
from console_task_urwid_functions import *
import rospy
from geometry_msgs.msg import PoseStamped, Point
from flyingros_libs.algorithm_functions import deg2radf, rad2degf
from flyingros_libs.transformations import euler_from_quaternion

class Object(object):
    def __init__(self, a):
        self.a =  a
        self.uniqueID = id(self)
    def __str__(self):
        return str(self.a)

arrayData = list()
arrayData.append(Object("coucou"))
arrayData.append(Object("Mamamamamamma"))
arrayData.append(Object("qmsldkqmlskdmlqksd"))
arrayData.append(Object("kk"))
arrayData.append(Object("qlsdkjqlksdj"))


def local_position_callback(data):
    # Input data : local position
    # Output data
    global local_pose, yaw
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    _,_,ryaw = euler_from_quaternion(quaternion, axes="sxyz")

    local_pose = data.pose.position
    yaw = rad2degf(ryaw)

def init():
    global local_pose, yaw
    local_pose = Point()
    yaw = 0
    pass

def subscribers():
    rospy.init_node('console_task')

    local_position_sub  = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_position_callback)

def main():
    pass

if __name__ == '__main__':
    rospy.loginfo("console task started")
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init console task started failed")
        pass
