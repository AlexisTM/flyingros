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

#include <Transform.h>
#include <tf/transform_datatypes.h>
#include "flyingros_msgs/Distance.h"
#include "geometry_msgs/Vector3.h"

namespace flyingros_pose
{
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
      LaserProjector(tf::Vector3 _position, tf::Vector3 _orientation, double _offset) {
        position = _position;
        orientation = _orientation.normalize();
        offset = _offset;
      };

      // To deallocate the object
      ~LaserProjection(){};

      // Update the laser orientation quaternion
      void updateOrientation(tf::Vector3 _orientation);

      // Project the laser and returns the actual true measure
      double projectLaser(double _measure, tf::Quaternion _q);

      // Laser position
      tf::Vector3 position;
      // Laser normalized orientation
      tf::Vector3 orientation;
      // Laser offset
      double offset;
  }
}


#endif //LASER_ALGORITHM_FUNCTIONS
