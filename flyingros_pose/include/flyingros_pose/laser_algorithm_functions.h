/*
 * laser_algorithm_functions.cpp
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

#ifndef LASER_ALGORITHM_FUNCTIONS_H
#define LASER_ALGORITHM_FUNCTIONS_H

#include "tf/transform_datatypes.h"
//#include <tf/transform_datatypes.h>
#include <cmath>
#include "flyingros_msgs/Distance.h"
//#include "geometry_msgs/Vector3.h"
#include <iostream>

#define M_PI180 3.14159265358979323846/180
#define M_180PI 180/3.14159265358979323846

namespace flyingros_pose
{
   // Convert degrees to radians
   double inline deg2radf(double angle){
       return double(angle)*M_PI180;
   }

   // Convert radians to degrees
   double inline rad2degf(double angle){
       return double(angle)*M_180PI;
   }

  //! \brief Laser represents a laser
  /*!
   * This class provide a laser which cache the laser position/orientation to
   * make a more efficient laser projector.
   *
   * The Laser class aims to get the laser projection.
   */
  class Laser {
  public:
    // Initialisation
    Laser(tf::Vector3 _position, tf::Vector3 _orientation, double _offset) {
      position = _position;
      if(_orientation == tf::Vector3(0,0,0)) // This is bad... unhandled... division by 0
         _orientation = tf::Vector3(1,0,0);
      orientation = _orientation.normalize();
      offset = _offset;
    };

    // To deallocate the object
    ~Laser(){};

    // Update the laser orientation quaternion
    void updateOrientation(tf::Vector3 _orientation){
      orientation = _orientation.normalize();
    }

    // Project the laser and returns the actual true measure
    tf::Vector3 project(double _measure, tf::Quaternion _q){
      _measure = _measure - offset;
      tf::Vector3 r_orientation = tf::quatRotate(_q, orientation);
      tf::Vector3 r_position = tf::quatRotate(_q, position);
      return _measure*r_orientation + r_position;
    }

    // Last result
    tf::Vector3 last;
    // Laser position
    tf::Vector3 position;
    // Laser normalized orientation
    tf::Vector3 orientation;
    // Laser offset
    double offset;
  };

  // Remove YAW component from quaternion with SRPY convention
  // Could be pimped
  tf::Quaternion nullYawQuaternion(tf::Quaternion q){
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return tf::createQuaternionFromRPY(roll, pitch, 0);
  }
  
  // Rotate lasers and project measures to the wall.
  // Finaly, compute yaw angle to the wall
  // _laser1 & _laser2 are the lasers
  // _measure1 & _measure2 are the measures of the precedent lasers
  // _q is the quaternion rotation with only roll & pitch
  // index1 & index2 determines if we use X, Y or Z values for numerator or denominator
  // atan2(a(index1),b(index2))
  double getYawFromTargets(tf::Vector3 target1, tf::Vector3 target2, int index1 = 0, int index2 = 1){
    // tf::Vector3::m_floats[0] = x
    // tf::Vector3::m_floats[1] = y
    // tf::Vector3::m_floats[2] = z
    double a = target2.m_floats[index1] - target1.m_floats[index1];
    double b = target2.m_floats[index2] - target1.m_floats[index2];
    double result = atan2(a,b);
    std::cout << "atan2(" << a << "," << b << ") = " << result << std::endl;
    return result;
  }

}


#endif //LASER_ALGORITHM_FUNCTIONS
