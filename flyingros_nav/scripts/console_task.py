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


#######  URWID  #######
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

class PopupAddTask(urwid.WidgetWrap):
    """A dialog that appears with nothing but a close button """
    signals = ['close']
    def __init__(self, on_press_function):
        add_button = urwid.Button("Add", on_press=on_press_function)
        cancel_button = urwid.Button("X Cancel", on_press=lambda button:self._emit("close"))

        pile = urwid.Pile([urwid.Text(
            "^^  I'm attached to the widget that opened me. "
            "Try resizing the window!\n"),
            urwid.GridFlow([add_button, cancel_button],15, 3, 1, 'center')
            ])
        fill = urwid.Filler(pile)
        self.__super.__init__(urwid.AttrWrap(fill, 'popbg'))

class ButtonWPopup(urwid.PopUpLauncher):
    def __init__(self, label, color, PopUpClass, on_press):
        self.__super.__init__(urwid.AttrWrap(urwid.Button(label, on_press=lambda button: self.open_pop_up()),color))
        self.on_press = on_press
        self.PopClass = PopUpClass
    def create_pop_up(self):
        pop_up = PopupAddTask(self.on_press)
        urwid.connect_signal(pop_up, 'close',
            lambda button: self.close_pop_up())
        return pop_up

    def get_pop_up_parameters(self):
        return {'left':0, 'top':1, 'overlay_width':32, 'overlay_height':7}

class TaskWidget(urwid.WidgetWrap):
    def __init__ (self, id, task):
        self.id = id
        self.content = task
        self.item = [
            ('fixed', 12, urwid.Padding(urwid.AttrWrap(
                urwid.Text('task %s' % str(id)), 'body', 'focus'), left=2)),
            ('fixed', 12, urwid.AttrWrap(urwid.Text('%s' % str(task.uniqueID)), 'body', 'focus')),
            urwid.AttrWrap(urwid.Text('%s' % str(task.a)), 'body', 'focus'),
        ]
        w = urwid.Columns(self.item)
        self.__super.__init__(w)

    def selectable (self):
        return True

    def keypress(self, size, key):
        return key

def add_tasks(button):

    # print button
    pass

def keyboard_handler(key):
    global taskListbox, tasksWidget
    if key in  ('a', 'A'):
        pass
    elif key == 'enter':
        pass
    elif key in  ('q', 'Q', 'ESC'):
        raise urwid.ExitMainLoop()

def main_urwid():
    global taskListbox, tasksWidget

    palette = [
        (None,  'white', 'black'),
        ('focus', 'white', 'dark blue'),
        ('body', 'light gray', 'black'),
        ('taskheader', 'black', 'light green'),
        ('header', 'black', 'light cyan'),
        ('button', 'black', 'light gray')]

    tasksWidget = list()
    for id, task in enumerate(arrayData):
        tasksWidget.append(TaskWidget(id, task))

    taskWalker = urwid.SimpleListWalker(tasksWidget)
    taskListbox = urwid.ListBox(taskWalker)

    # commandWalker =  urwid.SimpleListWalker(list(urwid.Text('I am left dabedi dabeda')))
    # commandListbox = urwid.ListBox(commandWalker)

    taskHeader = urwid.AttrWrap(urwid.Text(["Task list"],  align='center'), 'taskheader')
    taskView = urwid.Frame(urwid.AttrWrap(taskListbox, 'body'), header=taskHeader)

    ### Control widgets
    blank = urwid.Divider()


    control_content = [
        urwid.Text("This is a content"),
        urwid.Text("This is another content"), blank,blank,

        urwid.GridFlow(
            [ButtonWPopup("Add a task pop",'button', add_tasks, PopupAddTask),
            urwid.AttrWrap(urwid.Button("Add a task", on_press=add_tasks), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button')]
        ,15, 3, 1, 'center')
    ]

    controlListBox = urwid.ListBox(control_content)

    controlHeader = urwid.AttrWrap(urwid.Text(["Controller"],  align='center'), 'header')
    controlView = urwid.Frame(urwid.AttrWrap(controlListBox, 'body'), header=controlHeader)
    #view = urwid.Frame(urwid.AttrWrap(columns, 'body'), header=header)

    columns = urwid.Columns([controlView, taskView])
    loop = urwid.MainLoop(columns, palette, unhandled_input=keyboard_handler)
    loop.run()

#######  ROS  #######
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
    main_urwid()
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
