#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
mavros_quaternion_test.py

It is designed simply to test the quaternion received by the Pixhawk.
"""


from __future__ import division
import rospy

from flyingros_msgs.msg import Distance
from sensor_msgs.msg import Imu
from flyingros_libs.transformations import *
from flyingros_libs.algorithm_functions import deg2radf, rad2degf, quaternionRotation, extrapolate
from flyingros_libs.getch import *

def imu_callback(data):
    global q_imu
    q_imu = data.orientation

def logit():
    global q_imu
    global fake_measure
    Q_imu = (q_imu.x, q_imu.y, q_imu.z, q_imu.w)
    roll, pitch, yaw = euler_from_quaternion(Q_imu, axes="sxyz")
    pitch = -pitch
    rospy.loginfo("roll {0}, pitch {1}, yaw {2} (deg)".format(round(rad2degf(roll),2), 
                                                                  round(rad2degf(pitch),2), 
                                                                  round(rad2degf(yaw),2)))
    target = list()

    for measure in fake_measure:
        vertical_position = quaternionRotation([1.0,1.0,1.0], Q_imu)
        vertical_orientation = quaternionRotation([0.0,0.0,-1.0], Q_imu)
        target.append(extrapolate([0.0, 0.0, 0.0], vertical_orientation, measure))

    rospy.loginfo("target : {0} \n".format(target))

def quaternion_test():
    global q_imu
    Q_imu = (q_imu.x, q_imu.y, q_imu.z, q_imu.w)
    roll, pitch, yaw = euler_from_quaternion(Q_imu, axes="sxyz")
    Q_c = quaternion_from_euler(roll, -pitch, yaw, axes="sxyz")

    rospy.loginfo("Q   x {0}, y {1}, z {2}, w {3}".format(round(Q_c[0],2), 
                                                      round(Q_c[1],2), 
                                                      round(Q_c[2],2), 
                                                      round(Q_c[3],2)))

    rospy.loginfo("IMU x {0}, y {1}, z {2}, w {3}".format(round(Q_imu[0],2), 
                                                      round(Q_imu[1],2), 
                                                      round(Q_imu[2],2), 
                                                      round(Q_imu[3],2)))

def main():
    while True:
        what = getch()
        if what in ["q", "Q", 'a', 'A']:
            break
        if what in ['d', 'D']:
            logit()
        if what in ['f', 'F']:
            quaternion_test()

def init():
    global q_imu
    global fake_measure, fake_distance

    q_imu = Imu().orientation

    fake_distance = [3.0, 6.0, 0.0, 0.0, 0.0, 0.0]
    fake_measure = [3.0, 6.0, 0.0, 0.0, 0.0, 0.0]

    rospy.init_node('quaternion_test')
    imu_sub         = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)

if __name__ == '__main__':
    rospy.loginfo("quaternion_test started")
    try:
        init()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("quaternion_test failed")
        pass