#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tasks.py

This is a task implementation initialy used to control an UAV

This file is part of FlyingROS.

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

import rospy
import time
from math import fabs
from threading import Thread
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point, Vector3, TwistStamped
from flyingros_msgs.msg import Task
from transformations import *
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool
from algorithm_functions import deg2radf, rad2degf

def default_if_zero(data, default=0):
    return data if(data != None and data != 0) else default

def pythontask_to_rostask(python_task):
    if(python_task.getType() == "target"):
        return Task(python_task.name,
            Task.TYPE_TARGET,
            python_task.target,
            rad2degf(python_task.orientation),
            [python_task.precision.x, python_task.precision.z, python_task.precisionYAW],
            python_task.ID)

    elif(python_task.getType() == "loiter"):
        return Task(python_task.name,
            Task.TYPE_LOITER,
            Point(0,0,0),
            0,
            [python_task.waitTime],
            python_task.ID)

    elif(python_task.getType() == "takeoff"):
        return Task(python_task.name,
            Task.TYPE_TAKEOFF,
            Point(0,0,0),
            0,
            [python_task.precision],
            python_task.ID)

    elif(python_task.getType() == "land"):
        return Task(python_task.name,
            Task.TYPE_LAND,
            Point(0,0,0),
            0,
            [python_task.precision],
            python_task.ID)

    elif(python_task.getType() == "grab"):
        return Task(python_task.name,
            Task.TYPE_GRAB,
            Point(0,0,0),
            0,
            [python_task.state],
            python_task.ID)

    elif(python_task.getType() == "init_UAV"):
        return Task(python_task.name,
            Task.TYPE_INIT_UAV,
            Point(0,0,0),
            0,
            [float(python_task.sleep)],
            python_task.ID)

    elif(python_task.getType() == "arm"):
        return Task(python_task.name,
            Task.TYPE_ARM,
            Point(0,0,0),
            0,
            [float(python_task.timeout)],
            python_task.ID)

    elif(python_task.getType() == "disarm"):
        return Task(python_task.name,
            Task.TYPE_DISARM,
            Point(0,0,0),
            0,
            [float(python_task.timeout)],
            python_task.ID)
    else :
        return Task("null",
            Task.TYPE_NULL,
            Point(0,0,0),
            0,
            [],
            python_task.ID)

def rostask_to_pythontask(ros_task):
    python_task = object()
    if(ros_task.mission_type == Task.TYPE_TARGET):
        python_task = target(  ros_task.name,
                        ros_task.position,
                        ros_task.yaw,
                        default_if_zero(ros_task.data[0], 0.05),
                        default_if_zero(ros_task.data[1], 0.05),
                        default_if_zero(ros_task.data[2], 1))

    elif(ros_task.mission_type == Task.TYPE_ARM):
        python_task = arm( ros_task.name,
                        default_if_zero(ros_task.data[0],1))

    elif(ros_task.mission_type == Task.TYPE_DISARM):
        python_task = disarm( ros_task.name,
                        default_if_zero(ros_task.data[0],1))

    elif(ros_task.mission_type == Task.TYPE_LOITER):
        python_task = loiter(  ros_task.name,
                        default_if_zero(ros_task.data[0],1))

    elif(ros_task.mission_type == Task.TYPE_TAKEOFF):
        python_task = takeoff( ros_task.name,
                        default_if_zero(ros_task.data[0],0.05))

    elif(ros_task.mission_type == Task.TYPE_LAND):
        python_task = land( ros_task.name,
                        default_if_zero(ros_task.data[0],0.05))

    elif(ros_task.mission_type == Task.TYPE_INIT_UAV):
        python_task = init_UAV( ros_task.name,
                        default_if_zero(ros_task.data[0],5))

    elif(ros_task.mission_type == Task.TYPE_GRAB):
        python_task = grab( ros_task.name,
                     default_if_zero(ros_task.data[0],1.0))
    else:
        python_task = task()

    python_task.ID = ros_task.ID
    return python_task
    #elif(ros_task.mission_type == Task.TYPE_TEST):
    #    pass

class UAV:
    def __init__(self, setpoint_rate=10):
        # Configurations :
        self.type_mask_Fly = 2552 # 2552 - 0000 1001 1111 1000, position setpoint + Pxyz Yaw
        self.type_mask_Takeoff = 6599 # 6599 - 0001 1001 1100 0111, Takeoff setpoint + Vxyz Yaw
        self.type_mask_Land = 10695 # 10695 - 0010 1001 1100 0111, Land setpoint + Vxyz Yaw
        # Could be :
        # self.type_mask_Land = 10688 # 10695 - 0010 1001 1100 0000, Land setpoint + Pxyz Vxyz Yaw
        self.type_mask_Loiter = 14784 # 14784 - 0011 1001 1100 0000, Loiter setpoint + Pxyz Vxyz Yaw

        # Variable initiating
        self.state = State()
        self.local_position = Point()
        self.local_yaw = 0.0
        self.laser_position = Point()
        self.laser_yaw = 0.0
        self.stopped = False
        self.flying = True
        self.home = Point()
        self.quaternion = Quaternion()

        # Configurations
        self.landing_speed = -0.1 # 0.1 meters/s to go down
        self.landing_altitude = 0.10 # At 0.1 meters, shutdown motors, you are done
        self.takeoff_speed = 2 # 0.5 meters/s to get up
        self.takeoff_altitude = 0.50 # 0.5 meters to takeoff, once there, takeoff is succeeded

        # PixHawk position subscriber
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)

        # State subscriber
        self.state_subscriber = rospy.Subscriber('mavros/state', State, self.state_callback)

        # Arming & mode Services
        if not test :
            rospy.loginfo("Subscribing to arming service")
            rospy.wait_for_service('mavros/cmd/arming')
            self.arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            rospy.loginfo("Subscribing to setmode service")
            rospy.wait_for_service('mavros/set_mode')
            self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Setpoints
        self.setpoint_init()
        self.setpoint_rate = rospy.Rate(setpoint_rate)
        #self.setpoint_subscriber = rospy.Subscriber('/UAV/Setpoint', PoseStamped, self.setpoint_callback)
        self.laser_position_sub  = rospy.Subscriber('/lasers/filtered', PoseStamped, self.laser_position_sender)
        self.setPointsCount = 0
        # Senders threads
        self.setpoint_thread = Thread(target=self.setpoint_sender).start()
        self.laser_position_count = 0

    def setpoint_position(self, position, yaw):
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.velocity = Vector3()
        self.setpoint.position = position
        self.setpoint.yaw = yaw

    def setpoint_takeoff_here_position(self, altitude):
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.velocity = Vector3()
        self.setpoint.position = self.position
        self.setpoint.position.z = altitude
        self.setpoint.yaw = self.local_yaw

    def setpoint_land_here_position(self):
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.velocity = Vector3()
        self.setpoint.position = self.position
        self.setpoint.position.z = 0.3
        self.setpoint.yaw = self.local_yaw

    def setpoint_takeoff(self):
        self.setpoint.type_mask = self.type_mask_Takeoff
        self.setpoint.velocity = Vector3(0.0,self.takeoff_speed,0.0)

    def setpoint_land(self):
        self.setpoint.type_mask = self.type_mask_Land
        self.setpoint.velocity = Vector3(0.0,self.landing_speed,0.0)

    def setpoint_loiter(self):
        self.setpoint.type_mask = self.type_mask_Loiter
        self.setpoint.velocity = Vector3(0.0,0.0,0.0)

    def setpoint_init(self):
        # type_mask
        # 2552 : XYZ & yaw - POSITION
        # 7104 : XYZ, yaw, vXYZ, TAKE_OFF_SETPOINT
        # 3064 : 0000 1001 1111 1000
        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = self.setpoint.FRAME_LOCAL_NED
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.position = Point()
        self.setpoint.yaw = 0.0
        self.setpoint.velocity = Vector3()
        self.setpoint.acceleration_or_force = Vector3()
        self.setpoint.yaw_rate = 0.0

        self.setpoint_position = PoseStamped()
        self.laser_position_count = 0

    def local_position_callback(self, local):
        self.quaternion = local.pose.orientation
        self.local_position = Point(local.pose.position.x, local.pose.position.y,local.pose.position.z)
        q = (local.pose.orientation.x, local.pose.orientation.y, local.pose.orientation.z, local.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(q, axes="sxyz")
        self.local_yaw = yaw

    def state_callback(self, state):
        self.state = state

    # def setpoint_callback(self, setpoint):
    #     self.setpoint = setpoint

    def laser_position_sender(self, data):
        laser_pos_pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
        while(not self.stopped):
            data.header.stamp = rospy.Time.now()
            data.header.seq=self.positionCount
            self.positionCount = self.positionCount + 1
            laser_pos_pub.publish(data)
            self.setpoint_rate.sleep()

    def setpoint_sender(self):
        # setpoint_raw
        setpoint_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        while(not self.stopped):
            self.setpoint.header.stamp = rospy.Time.now()
            self.setpoint.header.seq=self.setPointsCount
            self.setPointsCount = self.setPointsCount + 1
            setpoint_publisher.publish(self.setpoint)
            self.setpoint_rate.sleep()

        # setpoint position only
        # setpoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        # while(not self.stopped):
        #     self.setpoint_position.header.stamp = rospy.Time.now()
        #     self.setpoint_position.header.stamp = self.laser_position_count
        #     self.setpoint_position.pose.orientation = self.quaternion
        #     self.setpoint_position.pose.position = self.setpoint.position
        #     setpoint_publisher.publish(self.setpoint)
        #     self.laser_position_count += 1
        #     self.setpoint_rate.sleep


    def getPosition(self):
        return [self.local_position, self.local_yaw]

    def arm(self, arming_state):
        self.arming_client(arming_state)

    def set_offboard(self):
        self.set_mode_client(custom_mode = "OFFBOARD")

    def die(self):
        self.setpoint_land()
        time.sleep(3)
        self.stopped = True
        time.sleep(1)
        self.arm(False)

    def __del__(self):
        self.die()

    def __exit__(self):
        self.die()


class taskController:
    """ The task controller handle a list with every tasks """
    def __init__(self, rate=10, setpoint_rate=10, test=False):
        self.tasks = list()
        self.count = 0
        self.current = 0
        self.setRate(rate)
        self.UAV = UAV(setpoint_rate=setpoint_rate)

    def __str__(self):
        controller_string = "Task Controller :\n"
        for task in self.tasks:
            controller_string += task.__str__()
            controller_string += "\n"

    def addTask(self, task):
        task.updateID()
        self.tasks.append(task)
        self.count += 1
        return self.count-1

    # returns the ID of the delete object or None
    def removeTask(self, taskID):
        for i, t in enumerate(self.tasks):
            if t.ID == taskID:
                self.tasks.pop(i)
                self.count -= 1
                return i
        return None

    def addTasks(self, tasks):
        for task in tasks:
            task.updateID()
            self.tasks.append(task)
        self.count += len(tasks)
        return self.count-1

    def getTasks(self):
        return self.tasks

    def getTask(self, index):
        # if index = 5, it is the last of a 6 task list
        return task("Null", "not_found") if (index > self.count-1) else self.tasks[index]

    def getCurrentTask(self):
        if self.current <  self.count :
            return self.tasks[self.current]
        else :
            return None

    def runTask(self):
        if self.current == -1 :
            rospy.loginfo("No task running (-1)")
        else :
            if self.current <  self.count :
                rospy.loginfo("Running task {0}/{1} - {2}".format(self.current+1, self.count, self.tasks[self.current]))
            else :
                rospy.loginfo("Out of task {0}/{1}".format(self.current+1, self.count))

    def setRate(self, rate):
        self.rate = rospy.Rate(rate)

    def getUAV(self):
        return self.UAV

    def spinOnce(self):
        if self.current < self.count :
            task = self.tasks[self.current]
            result = task.run(self.UAV)
            if result: # returns True if done
                self.current = self.current + 1
                self.runTask()
        return

    def __del__(self):
        self.UAV.__del__()
        del self.UAV

    def __exit__(self):
        self.UAV.__exit__()
        del self.UAV

class task:
    """The Task class defines a class & the needed methods to
    be compatible with the taskController"""
    def __init__(self, Type = "null", name = "null"):
        self.name = name
        self.Type = Type
        self.done = False
        self.ID = None

    def updateID(self):
        self.ID = id(self)

    def isDone(self):
        # check if task is ended
        # return True if done, False if not done
        return True

    def run(self, UAV):
        # do something then return isDone
        return self.isDone()

    def getType(self):
        return self.Type

    def __str__(self):
        return "Task {0} - {1}".format(self.Type, self.name)


# Go to a certain position
# Tested, working
class target(task, object):
    """The target class is a task. It says to the UAV to go to
    the target"""
    def __init__(self, name, pointXYZ, yaw = 0, precisionXY = 0.05, precisionZ = 0.05, precisionYAW = 1):
        super(target, self).__init__("target", name)
        self.target       = pointXYZ
        self.orientation  = deg2radf(yaw)
        self.precision    = Point(precisionXY, precisionXY, precisionZ)
        self.precisionYAW = precisionYAW
        self.sent         = False

    def __str__(self):
        return super(target, self).__str__() + "{0} pointing {1} radians".format(self.target, self.orientation)

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_position(self.target, self.orientation)
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Method to know if the UAV arrived at his position or not

        Better solution for more complex surfaces :
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        rospy.loginfo("X %s, prec : %s", UAV.ition.x - self.target.x, self.precision.x)
        rospy.loginfo("Y %s, prec : %s", UAV.local_position.y - self.target.y, self.precision.y)
        rospy.loginfo("Z %s, prec : %s", UAV.local_position.z - self.target.z, self.precision.z)
        rospy.loginfo("yaw: %s", UAV.local_yaw)
        if(fabs(UAV.local_position.x - self.target.x) > self.precision.x):
            return False
        if(fabs(UAV.local_position.y - self.target.y) > self.precision.y):
            return False
        if(fabs(UAV.local_position.z - self.target.z) > self.precision.z):
            return False
        if(fabs(UAV.local_yaw - self.orientation) > self.precisionYAW):
            return False
        # it is done
        return True


# Sleeping the right time
# Tested, working
class loiter(task, object):
    """The loiter class is a task. It aims to loiter for X seconds"""
    def __init__(self, name, waitTime):
        self.waitTime = waitTime
        self.start = None
        self.end = None
        super(loiter, self).__init__("loiter", name)

    def __str__(self):
        return super(loiter, self).__str__()

    def run(self, UAV):
        if(self.start == None):
            self.start = rospy.Time.now().to_sec()
            self.end = self.start + self.waitTime
            UAV.setpoint_loiter()
        return self.isDone()

    def isDone(self):
        now = rospy.Time.now().to_sec()
        if rospy.Time.now().to_sec() > self.end:
            return True
        return False

# Sleeping the right time
# NOT WORKING
class loiter_callback(task, object):
    """The loiter_callback class is a task. It aims to loiter for X seconds"""
    def __init__(self, name, waitTime):
        self.duration = rospy.Duration(waitTime)
        self.called = False
        self.done = False
        super(loiter_callback, self).__init__("loiter_callback", name)

    def __str__(self):
        return super(loiter_callback, self).__str__()

    def callbackTimer(self, event):
        self.done = True

    def run(self, UAV):
        if(self.called):
            rospy.Timer(self.duration, self.callbackTimer)
            self.called = True
        return self.isDone()

    def isDone(self):
        return self.done

class takeoff(task, object):
    """The takeoff class is a task. It says to the UAV to go to
    takeoff"""
    def __init__(self, name, precision=0.05):
        super(takeoff, self).__init__("takeoff", name)
        self.sent             = False
        self.takeoff_altitude = 1
        self.precision        = precision

    def __str__(self):
        return super(takeoff, self).__str__() + "TakeOff"

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_takeoff()
            # UAV.setpoint_takeoff_here_position(self.takeoff_altitude)
            self.takeoff_altitude = UAV.takeoff_altitude
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Just verify if we correctly take off or not """
        if(fabs(UAV.local_position.z - self.takeoff_altitude) > self.precision):
            return False
        # it is done
        return True


class land(task, object):
    """The land class is a task. It says to the UAV to go to
    land"""
    def __init__(self, name, precision=0.05):
        super(land, self).__init__("land", name)
        self.sent             = False
        self.landing_altitude = 0.2
        self.precision        = precision

    def __str__(self):
        return super(land, self).__str__() + "Landing"

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_land()
            #UAV.setpoint_land_here_position()
            self.landing_altitude = UAV.landing_altitude
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Just verify if we correctly landed or not, maximum landing_altitude + precision (15cm) """
        if(fabs(UAV.local_position.z - self.landing_altitude) > self.precision):
            return False
        # it is done
        return True


""" To be implemented """
class grab(task, object):
    """The grab class is a task. It says to the UAV to grab or realease
    the pliers"""
    def __init__(self, name, state):
        super(grab, self).__init__("grab", name)
        self.state = state;

    def __str__(self):
        return super(grab, self).__str__() + "Grab"

    def run(self, UAV):
        return self.isDone()

    def isDone(self):
        """ Method to know if the UAV arrived at his position or not
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        return True



# Waiting testing time
class test(task, object):
    """The test class is a task. It is just a testing task"""
    def __init__(self, name, waitTime):
        super(test, self).__init__("test", name)
        self.waitTime = waitTime
        self.last = None

    def __str__(self):
        return super(test, self).__str__() + "test"

    def run(self, UAV):
        if(self.last == None):
            self.last = time.time()
            print("first time")
        return self.isDone()

    def isDone(self):
        now = time.time()
        print(now - self.last)

        #Simplify by using :
        # return (now - self.last) > self.waitTime

        if(now - self.last > self.waitTime):
            print("done")
            return True
        else :
            print("waiting")
            return False

# Not optimized
class init_UAV(task, object):
    """The init_UAV class is a task. It wait the UAV to be initialized, set home and set the setpoint to hom"""
    def __init__(self, name, sleep = 5):
        self.sleep = sleep
        self.rate = rospy.Rate(1.0/sleep)
        super(init_UAV, self).__init__("init_UAV", name)

    def __str__(self):
        return super(init_UAV, self).__str__() + " init"

    def run(self, UAV):
        self.rate.sleep()
        UAV.home = Point(UAV.local_position.x, UAV.local_position.y, UAV.local_position.z)
        UAV.setpoint.position = Point(UAV.local_position.x, UAV.local_position.y, UAV.local_position.z)
        return self.isDone(UAV)

    def isDone(self, UAV):
        return True


# Arm the pixHawk
# Tested, working
class arm(task, object):
    """The arm class is a task. It put the UAV in OFFBOARD mode and arm it"""
    def __init__(self, name, timeout = 1):
        self.timeout = timeout
        self.rate = rospy.Rate(1.0/timeout)
        super(arm, self).__init__("arm", name)

    def __str__(self):
        return super(arm, self).__str__() + " arming"

    def run(self, UAV):
        if not UAV.state.armed :
            UAV.arm(True)
        self.rate.sleep()
        if not (UAV.state.mode == "OFFBOARD" or UAV.state.mode == "AUTO.LAND"):
            UAV.set_offboard()
        self.rate.sleep()
        return self.isDone(UAV)

    def isDone(self, UAV):
        return UAV.state.armed and ( UAV.state.mode == "OFFBOARD" or UAV.state.mode == "AUTO.LAND")


# Disarm the pixHawk
# Tested, working
class disarm(task, object):
    """The disarm Class disarms the UAV"""
    def __init__(self, name, timeout = 1):
        self.timeout = timeout
        self.rate = rospy.Rate(1.0/timeout)
        self.last = None
        super(disarm, self).__init__("disarm", name)

    def __str__(self):
        return super(disarm, self).__str__() + " disarming"

    def run(self, UAV):
        if UAV.state.armed :
            UAV.arm(False)
        self.rate.sleep()
        return self.isDone(UAV)

    def isDone(self, UAV):
        return not UAV.state.armed
