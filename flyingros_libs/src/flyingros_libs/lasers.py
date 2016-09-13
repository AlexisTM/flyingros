#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
lasers.py

This class represents the lasers real configuration to be used in the
algorithm. It allows to avoid to hardcode the positions.

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

from math import sqrt
import rospy
from transformations import *
from algorithm_functions import *

# offset is defined as
class laser:
    def __init__(self, position, orientation, offset):
        self.position = position
        self.orientation = orientation
        self.offset = offset
        self.length = sqrt(orientation[0]*orientation[0] + orientation[1]*orientation[1] + orientation[2]*orientation[2])


class lasersController:
    def __init__(self):
        self.X1 = laser((0.07,0.21,-0.04), (1,0,0), (1,-19))
        self.X2 = laser((-0.15,-0.21,-0.04), (1,0,0), (1,-1))
        self.Y1 = laser((0.0,-0.085,0.10), (0,1,0), (1,-6))
        self.Y2 = laser((-0.15,0.275,0.01), (0,1,0), (1,-9)) # NOT IN USE
        self.Z1 = laser((0.025,0,-0.015), (0,0,1), (1,0))
        self.Z2 = laser((-0.025,0,-0.015), (0,0,1), (1,0))
        self.list = (self.X1, self.X2, self.Y1, self.Y2, self.Z1, self.Z2)
        self.count = 6

    def target(self, q, raw):
        target = list()
        for i in range(6):
            position = quaternionRotation(self.list[i].position, q)
            orientation = quaternionRotation(self.list[i].orientation, q)
            target.append(extrapolate(position, orientation, raw[i]))
        return target

    def targetX(self, q, raw):
        target = list()
        for i in range(2):
            position = quaternionRotation(self.list[i].position, q)
            orientation = quaternionRotation(self.list[i].orientation, q)
            target.append(extrapolate(position, orientation, raw[i]))
        return target

    def preRotateX(self, q):
        # convert imu to a quaternion tuple
        laser1 = quaternionRotation(self.X1.position, q)
        orientation1 = quaternionRotation(self.X1.orientation, q)
        laser2 = quaternionRotation(self.X2.position, q)
        orientation2 = quaternionRotation(self.X2.orientation, q)
        return (laser1, orientation1, laser2, orientation2)


    def preRotateY(self, q):
        # convert imu to a quaternion tuple
        laser1 = quaternionRotation(self.Y1.position, q)
        orientation1 = quaternionRotation(self.Y1.orientation, q)
        laser2 = quaternionRotation(self.Y2.position, q)
        orientation2 = quaternionRotation(self.Y2.orientation, q)
        return (laser1, orientation1, laser2, orientation2)
