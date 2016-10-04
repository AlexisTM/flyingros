#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
control_tasks.py

This script sends positions to control the UAV in X, Y, Z using the
tasks controller

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

import rospy
import mavros
import time
from getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped, Point
from mavros.utils import *
from flyingros_libs.tasks import *
import sys

def interface_getch():
    global setpoint
    global Controller
    global stop

    what = getch()

    if what == "g":
        setpoint.x = setpoint.x + 0.1
    if what == "t":
        setpoint.x = setpoint.x - 0.1
    if what == "h":
        setpoint.y = setpoint.y + 0.1
    if what == "f":
        setpoint.y = setpoint.y - 0.1
    if what == "i":
        setpoint.z = setpoint.z + 0.1
    if what == "k":
        setpoint.z = setpoint.z - 0.1
    if what == "m":
        stop = True
        sys.exit("SHUTDOWN")


    task = Controller.getCurrentTask()
    #rospy.loginfo(task)

def main():
    do_job_thread = Thread(target=do_job).start()
    while not rospy.is_shutdown():
        interface_getch()


def do_job():
    # Input data
    global stop
    # Objects
    global Controller
    while not stop :
        Controller.rate.sleep()
        Controller.spinOnce()
    if stop :
        Controller.UAV.arm(False)

def task_feeder():
    global Controller
    Controller.addTask(init_UAV("Init", sleep=10))
    Controller.addTask(arm("Arming", timeout=1))
    time.sleep(10)
    position, yaw = Controller.UAV.getPosition()
    #Controller.addTask(takeoff("Takeoff"))
    Controller.addTask(target("Position 1 meter", Point(position.x, position.y, 1.0),0))
    Controller.addTask(loiter("Wait 5 seconds", 5))
    Controller.addTask(target("Position 1 meter", Point(position.x, position.y, 0.5),0))
    Controller.addTask(land("Landing incoming"))
    Controller.addTask(disarm("Shutdown now"))

def init():
    # Input data
    global stop
    # Objects
    global setpoint, Controller

    global uav
    # Data initiation
    stop = False
    setpoint = Point()

    # Node initiation
    rospy.init_node('laserpack_control')
    Controller = taskController(rate=3, setpoint_rate=10)

if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
        task_feeder()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
