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

void callback_laser_raw(const flyingros_msgs::Distance::ConstPtr& msg){
  double roll, pitch, yaw;

  double measures[6];
  for(int i = 0; i < 6; i++){
      // measures are in cm and have an offset
      measures[i] = double(lasers[i])/100.0 - lasers[i].offset;
  }

  tf::Matrix3x3 m(q_imu);
  m.getRPY(roll, pitch, yaw);
  tf::Quaternion q_zero = tf::createQuaternionFromRPY(roll, pitch, 0);

  tf::Vector3 targetx1 = laser[0].project(measures[0], q_zero);
  tf::Vector3 targetx2 = laser[1].project(measures[1], q_zero);
  double yaw_x = getYawFromTargets(targetx2, targetx1,0,1);

  tf::Vector3 targety1 = laser[2].project(measures[2], q_zero);
  tf::Vector3 targety2 = laser[3].project(measures[3], q_zero);
  double yaw_y = getYawFromTargets(targety2, targety1,1,0);

  tf::Quaternion q_correct = tf::createQuaternionFromRPY(roll, pitch, yaw_x);

  tf::Vector3 targetx1 = laser[0].project(measures[0], q_zero);
  tf::Vector3 targetx2 = laser[1].project(measures[1], q_zero);
  tf::Vector3 targety1 = laser[2].project(measures[2], q_zero);
  tf::Vector3 targety2 = laser[3].project(measures[3], q_zero);
  // get yaw
  // get position
  // publish
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){
    tf::quaternionMsgToTF(msg->orientation, q_imu);
}

void reconfigure_lasers(){
    XmlRpc::XmlRpcValue offsetsList, positionsList, orientationsList;
    XmlRpc::XmlRpcValue p, v;
    int count;
    ros::param::get("/flyingros/lasers/count", count);
    ROS_ASSERT(count == 6);
    ros::param::get("/flyingros/lasers/offsets", offsetsList);
    ROS_ASSERT(offsetsList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ros::param::get("/flyingros/lasers/positions", positionsList);
    ROS_ASSERT(positionsList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ros::param::get("/flyingros/lasers/orientations", orientationsList);
    ROS_ASSERT(orientationsList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    tf::Vector3 postition, orientation;
    double offset;
    for(int i = 0; i < count; i++){
        //cout << "Laser 1 : position - " <<  positionsList[i] << "  orientation - " << orientationsList[i] << "  offset - " << offsetsList[i] << endl;
        offset = offsetsList[i];
        p = positionsList[i];
        v = orientationsList[i];
        ROS_ASSERT(p.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);

        //ROS_ASSERT(p[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //ROS_ASSERT(p[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //ROS_ASSERT(p[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //ROS_ASSERT(v[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //ROS_ASSERT(v[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        //ROS_ASSERT(v[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
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

    std::string raw_laser_topic, position_pub_topic, imu_topic;
    ros::param::param<std::string>("laser_raw_topic", raw_laser_topic, "/flyingros/lasers/raw");
    ros::param::param<std::string>("laser_pose_topic", position_pub_topic, "/flyingros/lasers/pose");
    ros::param::param<std::string>("imu_topic", imu_topic, "/mavros/imu/data");

    ros::Subscriber raw_laser_sub = nh.subscribe(raw_laser_topic, 1, callback_laser_raw);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1, callback_imu);
    position_publisher = nh.advertise<geometry_msgs::Pose>(position_pub_topic, 1);

    reconfigure_lasers();
    //ros::spin();
    return 0;
}
