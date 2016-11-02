/*
 * altitude_lasers.cpp
 *
 * Altitude computation based on fixed lasers.
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

#include <typeinfo>
#include <iostream>
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include "flyingros_pose/laser_algorithm_functions.h"
#include "flyingros_msgs/Distance.h"

using namespace std;
using namespace flyingros_pose;

Laser lasers[6];
tf::Quaternion q_imu(0,0,0,1);
ros::Publisher position_publisher;
int laser_count;

void callback_laser_raw(const flyingros_msgs::Distance::ConstPtr& msg){
  double roll, pitch, yaw;

  // Correct offset
  double measures[6];
  for(int i = 0; i < laser_count; i++){
      // measures are in cm and have an offset
      measures[i] = double(msg->lasers[i])/100.0 - lasers[i].offset;
  }

  // Get position
  tf::Vector3 targets[6];
  for(int i = 0; i < laser_count; i ++){
    targets[i] = lasers[i].project(measures[i], q_imu);
  }

  double altitude = 0;
  for(int i = 0; i < laser_count; i ++){
    altitude += targets[i].z();
  }
  altitude = altitude/double(laser_count);

  // publish
  geometry_msgs::Pose UAVPose;
  tf::quaternionTFToMsg(q_imu, UAVPose.orientation);
  UAVPose.position.x = 0;
  UAVPose.position.y = 0;
  UAVPose.position.z = altitude;
  position_publisher.publish(UAVPose);
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){
    tf::quaternionMsgToTF(msg->orientation, q_imu);
}

void reconfigure_lasers(){
    XmlRpc::XmlRpcValue offsetsList, positionsList, orientationsList;
    XmlRpc::XmlRpcValue p, v;

    ros::param::get("/flyingros/lasers/count", laser_count);
    ROS_ASSERT(laser_count >= 1);
    ros::param::get("/flyingros/lasers/offsets", offsetsList);
    ROS_ASSERT(offsetsList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ros::param::get("/flyingros/lasers/positions", positionsList);
    ROS_ASSERT(positionsList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ros::param::get("/flyingros/lasers/orientations", orientationsList);
    ROS_ASSERT(orientationsList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    tf::Vector3 postition, orientation;
    double offset;
    for(int i = 0; i < laser_count; i++){
        offset = offsetsList[i];
        p = positionsList[i];
        v = orientationsList[i];
        ROS_ASSERT(p.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
        postition.setX(double(p[0]));
        postition.setY(double(p[1]));
        postition.setZ(double(p[2]));
        orientation.setX(double(v[0]));
        orientation.setY(double(v[1]));
        orientation.setZ(double(v[2]));
        lasers[i].configure(postition, orientation, offset);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_node_3D_algorithm_cpp");
    ros::NodeHandle nh;
    laser_count = 0;

    // Reconfigure laser values (from ROS parameters) before using them.
    reconfigure_lasers();

    std::string raw_laser_topic, position_pub_topic, imu_topic;
    ros::param::param<std::string>("laser_raw_topic", raw_laser_topic, "/flyingros/lasers/raw");
    ros::param::param<std::string>("laser_pose_topic", position_pub_topic, "/flyingros/lasers/pose");
    ros::param::param<std::string>("imu_topic", imu_topic, "/mavros/imu/data");

    ros::Subscriber raw_laser_sub = nh.subscribe(raw_laser_topic, 1, callback_laser_raw);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1, callback_imu);
    position_publisher = nh.advertise<geometry_msgs::Pose>(position_pub_topic, 1);

    ros::spin();
    return 0;
}
