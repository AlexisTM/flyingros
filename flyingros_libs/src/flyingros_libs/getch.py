#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
getch.py

Getch is an easy way to get a key in Python to have a minimal
lightweight interface with the user

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

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


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

            # [ Wait until ready for reading,
            #   wait until ready for writing
            #   wait for an "exception condition" ]
            # The below line times out after 1 second
            # This can be changed to a floating-point value if necessary
            [i, o, e] = select([sys.stdin.fileno()], [], [], 1)
            if i:
                ch = sys.stdin.read(1)
            else:
                ch = None

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return ch


class _GetchWindows:
    """Fetch a character using the Microsoft Visual C Runtime."""
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        import time

        # Delay timeout to match UNIX behaviour
        time.sleep(1)

        # Check if there is a character waiting, otherwise this would block
        if msvcrt.kbhit():
            return msvcrt.getch()

        else:
            return


getch = _Getch()
