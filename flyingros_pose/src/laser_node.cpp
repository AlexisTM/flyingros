/*
 * laser_node.cpp
 *
 * Functions used by the position algorithm based on laser projections.
 *
 * This file is a part of FlyingROS
 *
 * FlyingROS free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FlyingROS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FlyingROS.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Software created by Alexis Paques and Nabil Nehri for the UCL
 * in a Drone-Based Additive Manufacturing of Architectural Structures
 * project financed by the MIT Seed Fund
 *
 * Copyright (c) Alexis Paques 2016
 *
 */

#include <iostream>

#include "tf/transform_datatypes.h"
#include "flyingros_pose/laser_algorithm_functions.h"

using namespace std;
using namespace flyingros_pose;

int main()
{
    Laser laser1(tf::Vector3(0,0,0),tf::Vector3(1,0,0),0);
    Laser laser2(tf::Vector3(0.10,0.10,0.10),tf::Vector3(1,0,0),0);
    cout << "coucou" << endl;

    // @ 50Hz
    tf::Quaternion q_imu = tf::createQuaternionFromRPY(deg2radf(10),deg2radf(25),deg2radf(10));

    double roll, pitch, yaw;
    tf::Matrix3x3 m(q_imu);
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion q_zero = tf::createQuaternionFromRPY(roll, pitch, 0);

    tf::Vector3 thing1 = laser1.project(10, q_imu);
    tf::Vector3 thing2 = laser2.project(10, q_imu);

    double yaw_corrected = getYawFromTargets(thing1, thing2);
    cout << thing1.x() << " " << thing1.y() << " "<< thing1.z() << endl;
    cout << thing2.x() << " " << thing2.y() << " "<< thing2.z() << endl;
}
