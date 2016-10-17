#!/usr/bin/env python
PKG = 'flyingros_nav'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import logging
import rospy
#import flyingros_libs.tasks as tasks
from flyingros_libs.tasks import *
from flyingros_msgs.msg import Task
from geometry_msgs.msg import Point
#import geometry_msgs.msg as  geometry_m


#log= logging.getLogger( "TestTasksConversion.test_target" )
#log.debug( "this= %i", oros_task.ID )
#log.debug( "that= %i", iros_task.ID )

class TestTasksConversion(unittest.TestCase):
    def test_target(self):
        iros_task = Task("my name",
            Task.TYPE_TARGET,
            Point(1,2,3),
            1, # degrees
            [0.1,0.2,0.3],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_loiter(self):
        iros_task = Task("my name",
            Task.TYPE_LOITER,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_takeoff(self):
        iros_task = Task("my name",
            Task.TYPE_TAKEOFF,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_land(self):
        iros_task = Task("my name",
            Task.TYPE_LAND,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_grab(self):
        iros_task = Task("my name",
            Task.TYPE_GRAB,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_arm(self):
        iros_task = Task("my name",
            Task.TYPE_ARM,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

    def test_disarm(self):
        iros_task = Task("my name",
            Task.TYPE_DISARM,
            Point(),
            0, # degrees
            [12.1],
            42)

        oros_task = pythontask_to_rostask(rostask_to_pythontask(iros_task))
        self.assertEquals(iros_task, oros_task, "input and output ros_task equals")

## Testing test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest

    # Need ros for timing...
    rospy.init_node("test_node_time")

    #logging.basicConfig( stream=sys.stderr )
    #logging.getLogger( "TestTasksConversion.test_takeoff" ).setLevel( logging.DEBUG )
    #rostest.unitrun(PKG, 'unit_test_task', TestTasksConversion)
    rostest.unitrun(PKG, 'unit_test_task_conversion', TestTasksConversion)
    #import rostest
    #rostest.rosrun(PKG, 'test_bare_bones', TestTasksConversion)
