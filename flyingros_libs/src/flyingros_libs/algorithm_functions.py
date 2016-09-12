#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
algotithm_functions

This script have every background work functions for the position algorithm
algorithm.py

This file is part of ILPS (Indoor Laser Positioning System).

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

Copyright (c) Alexis Paques 2016
"""

from __future__ import division
from numpy import dot, pad, pi
from transformations import *
from math import sqrt, asin, acos, sin, cos, tan, atan, atan2
from time import time

# Returns radians from a degrees list
def deg2rad(arr):
	for i in xrange(len(arr)):
		arr[i] = float(arr[i])*pi/180
	return arr

# Returns degrees from a radians list
def rad2deg(arr):
	for i in xrange(len(arr)):
		arr[i] = float(arr[i])*180/pi
	return arr

# Returns a radian from a degree
def deg2radf(a):
	return float(a)*pi/180

# Returns a degree from a radian
def rad2degf(a):
	return float(a)*180/pi

# rotate a point, vector or quaternion by a quaternion
# The rotation DO NOT change the size of the distance to the origin
# input p : a tuple (x,y,z) or a quaternion (x,y,z,w)
# Create a matrix from the quaternion q and return the dot product of the vector/point/quaternion & p
def rotate(p, q):
    # if we got only a point/vector, convert it to (x,y,z,w), with w = 0
    if(len(p) == 3):
        p = p + (0,)
    m = quaternion_matrix(q)
    return dot(m, p)

# computes the X point where the laser points to on the well
# input p : point with the laser
# input v : the vector direction
# input M : distance measured
def extrapolate(p,v,M):
    # p is the origin
    # M = 20-5000 [cm]
    # find k, the factor to multiply v to get length(v) = M (interpolation)
    # Then add p to v to get the result
    K = sqrt(M*M/(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]))
    v = (K*v[0]+p[0], K*v[1]+p[1], K*v[2]+p[2])
    return v

# Get the length of the vector or the distance of the point to the origin
def length(point):
    return sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])

# input q : Quaternion
# output normalized quaternion
# It can normalize a vector
def normalizeQ(q):
    l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if(l == 1 or l == 0):
        return q
    return (q[0]/l, q[1]/l, q[2]/l, q[3]/l)

# input q : Quaternion
# output axis (x,y,z) and angle
def Quaternion2AxisAngle(q):
    q = normalizeQ(q)
    angle = 2 * acos(q[3]);
    s = sqrt(1-q[3]*q[3]);
    if(s <= 0.001):
        return (q[0], q[1], q[2]), angle
    return (q[0]/s, q[1]/s, q[2]/s), angle


# input q : Quaternion
# output axis (x,y,z) and angle
def fastQuaternion2AxisAngle(q):
    l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    q3 = q[3]/l
    angle = 2 * acos(q3/l);
    s = sqrt(1-q3*q3);
    if(s <= 0.001):
        return (q[0], q[1], q[2]), angle
    return (q[0]/s, q[1]/s, q[2]/s), angle

# CARE : Counterclock rotation whenn u point towards observer
# input vector3
# input axis (x,y,z) NORMALIZED
# input angle NORMALIZED
def rotationAxisAngle(vector, axis, angle):
    cost = cos(angle)
    omcost = 1-cost
    sint = sin(angle)
    return ((omcost*axis[0]*axis[2] + sint*axis[1])*vector[2] + (-sint*axis[2] + omcost*axis[0]*axis[1])*vector[1] + (omcost*axis[0]*axis[0]  + cost)*vector[0], \
    (omcost*axis[1]*axis[2] - sint*axis[0])*vector[2] + (omcost*axis[1]*axis[1]  + cost)*vector[1] + (omcost*axis[0]*axis[1] + sint*axis[2])*vector[0], \
    (omcost*axis[2]*axis[2]  + cost)*vector[2] + (omcost*axis[1]*axis[2] + sint*axis[0])*vector[1] + (omcost*axis[0]*axis[2] - sint*axis[1])*vector[0])

# Rotate a quaternion via the Axis Angle rotation
def quaternionRotation(p, q):
    axis, angle = Quaternion2AxisAngle(q)
    return  rotationAxisAngle(p, axis, angle)

# get distance of two 3x1 vectors
def distance(a,b):
    return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]))

# Rotate about Z, it is no more used
def rotateAboutZ(vector, angle):
    cost = cos(angle)
    sint = sin(angle)
    return (-sint*vector[1] + cost*vector[0], cost*vector[1]+sint*vector[0], vector[2])

# getYawInX
# gives the yaw (rotation about Z) from the position and orientation and measure of the lasers IN X
def getYawInX(position1, orientation1, measure1, position2, orientation2, measure2):
    length1 = length(orientation1)
    length2 = length(orientation2)
    k1 = measure1/length1
    k2 = measure2/length2
    # Shouldn't it be ...[1]/...[0] (Y/X) instead of ...[0]/...[1] Sign ?
    numerator   = k2*orientation2[0] - k1*orientation1[0] + position1[0] - position2[0]
    denominator = k2*orientation2[1] - k1*orientation1[1] + position1[1] - position2[1]
    return atan2(numerator,denominator)

# getYawInXL
# gives the yaw (rotation about Z) from the position and orientation and measure of the lasers IN X, knowing
# the length of the orientations
def getYawInXL(position1, orientation1, measure1, length1, position2, orientation2, measure2, length2):
    k1 = measure1/length1
    k2 = measure2/length2
    numerator   = k2*orientation2[0] - k1*orientation1[0] + position1[0] - position2[0]
    denominator = k2*orientation2[1] - k1*orientation1[1] + position1[1] - position2[1]
    #print "data : ", k2*orientation2[0], k1*orientation1[0], position1[0], position2[0]
    #print denominator
    return atan2(numerator,denominator)
