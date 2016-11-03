#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
urwid_app

urwid class with the application

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

Originaly published by Vladimir Ermakov (c) 2015 under GNU GPLv3
Copyright (c) Alexis Paques 2016
"""

import urwid

task_type = {
    0: "NULL",
    9:"INIT_UAV",
    13:"ARM",
    11:"DISARM",
    121:"LOITER",
    122:"TAKEOFF",
    123:"LAND",
    124:"TARGET",
    193:"GRAB",
    254:"TEST",
  }

class Object(object):
    def __init__(self, info):
      self.ID = info
      self.name = info
      return


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
    def __init__ (self, task):
        self.content = task
        self.item = [
            ('fixed', 12, urwid.Padding(urwid.AttrWrap(urwid.Text(' %s ' % str(task_type[task.mission_type])), 'body', 'focus'), left=2)),
            urwid.AttrWrap(urwid.Text('%s' % str(task.name)), 'body', 'focus'),
            urwid.AttrWrap(urwid.Text('%s' % str(task.position).replace('\n', ' ')), 'body', 'focus'),
            ('fixed', 10, urwid.AttrWrap(urwid.Text('%s' % str(task.ID)), 'body', 'focus'))
        ]
        w = urwid.Columns(self.item)
        self.__super.__init__(w)

    def selectable (self):
        return False

    def keypress(self, size, key):
        return key

class App(object):
  palette = [
    (None,  'white', 'black'),
    ('focus', 'white', 'dark blue'),
    ('body', 'light gray', 'black'),
    ('taskheader', 'black', 'light green'),
    ('header', 'black', 'light cyan'),
    ('button', 'black', 'light gray')]

  def __init__(self, update_mission=None, spin_switch=None):
    self.update_mission = update_mission or self.nothing
    self.spin_switch = spin_switch or self.nothing


    self.initAll()
    self.loop = urwid.MainLoop(self.columns, self.palette,
                                   unhandled_input=self._unhandled)
  
  def nothing(self):
    pass

  def _unhandled(self, key):
    if key in ('q', 'Q', 'ESC'):
      self.quit() 
    elif key in ('a', 'A'):
      self.taskWalker.append(TaskWidget(Object("coucou")))
    elif key in ('z', 'Z'):
      self.taskWalker[:] = [TaskWidget(Object("coucou")), TaskWidget(Object("kako"))]
    return

  def refresh_mission(self, mission):
    self.taskWalker[:] = []
    for id, task in enumerate(mission.tasks):
        self.taskWalker.append(TaskWidget(task))
    return

  def initAll(self):
    self.tasksWidget = list()
    self.taskWalker = urwid.SimpleListWalker(self.tasksWidget)
    self.taskListbox = urwid.ListBox(self.taskWalker)

    self.taskHeader = urwid.AttrWrap(urwid.Text(["Task list"],  align='center'), 'taskheader')
    self.taskView = urwid.Frame(urwid.AttrWrap(self.taskListbox, 'body'), header=self.taskHeader)

    ### Control widgets
    self.blank = urwid.Divider()

    self.spinningText = urwid.Text("SPINNING (is on automated task mode)", align='center')
    self.spinningButton = urwid.Button("Switch spinning OFF", self.spin_switch)


    self.control_content = [
        self.blank,
        self.spinningText,
        urwid.Text("This is another content"), self.blank,self.blank,

        urwid.GridFlow(
            [urwid.AttrWrap(urwid.Button("Update mission",  self.update_mission), 'button'),
            urwid.AttrWrap(self.spinningButton, 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button'),
            urwid.AttrWrap(urwid.Button("This is a button"), 'button')]
        ,15, 3, 1, 'center')]

    self.controlListBox = urwid.ListBox(self.control_content)
    self.controlHeader = urwid.AttrWrap(urwid.Text(["Controller"],  align='center'), 'header')
    self.controlView = urwid.Frame(urwid.AttrWrap(self.controlListBox, 'body'), header=self.controlHeader)
    #view = urwid.Frame(urwid.AttrWrap(columns, 'body'), header=header)

    self.columns = urwid.Columns([self.controlView, self.taskView])

  def setSpinning(self, spinning):
    if spinning : 
      self.spinningText.set_text("SPINNING (is on automated task mode)")
      self.spinningButton.set_label("Switch OFF spinning")
    else : 
      self.spinningText.set_text("NOT SPINNING (is on manual mode)")
      self.spinningButton.set_label("Switch ON spinning ")

  def quit(self):
    """Quit the program."""
    raise urwid.ExitMainLoop()

  def run(self):
    """Run the loop."""
    self.loop.run()