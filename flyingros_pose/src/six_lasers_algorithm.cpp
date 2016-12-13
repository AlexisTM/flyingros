/*
 * six_lasers_algorithm.cpp
 *
 * Localisation algorithm based on 6 fixed lasers.
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
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include "flyingros_pose/laser_algorithm_functions.h"
#include "flyingros_msgs/MultiEcho.h"
#include <ros/console.h> 
#include <cmath>

using namespace std;
using namespace flyingros_pose;

Laser lasers[6];
tf::Quaternion q_imu(0,0,0,1);
ros::Publisher pose_publisher;
ros::Publisher lowFrequency_publisher;
uint32_t sequence_count;
uint32_t lowf_sequence;
static tf::Vector3 lastPosition[3];
static int lastPositionIndex = 0; // 0,1,2
static geometry_msgs::PoseStamped UAVPose;
static geometry_msgs::PoseStamped UAVPoseLowf;

void filterPosition(tf::Vector3 * p)
{
  int index = lastPositionIndex;

  *p = lastPosition[index]*0.6;

  index -= 1;
  if(index < 0){
    index = 2;
  }

  *p += lastPosition[index]*0.3;

  index -= 1;
  if(index < 0){
    index = 2;
  }

  *p += lastPosition[index]*0.1;
}


void lowFrequency_pub(){
  UAVPoseLowf.pose.orientation = UAVPose.pose.orientation;
  UAVPoseLowf.header.seq = lowf_sequence;
  UAVPoseLowf.header.stamp = ros::Time::now();  

  tf::Vector3 filteredPosition;
  filterPosition(&filteredPosition);

  UAVPoseLowf.pose.position.x = filteredPosition.x();
  UAVPoseLowf.pose.position.y = filteredPosition.y();
  UAVPoseLowf.pose.position.z = filteredPosition.z();
  
  lowFrequency_publisher.publish(UAVPoseLowf);
  lowf_sequence++;
}

bool laserFailing(int measure){
  return measure > 900;
}

void callback_laser_raw(const flyingros_msgs::MultiEcho::ConstPtr& msg){
  double roll, pitch, yaw;
  static bool fails[6];
  static bool yawfails[2];

  for(int i = 0; i < 6; i++){
    fails[i] = laserFailing(msg->measures[i]);
  }
  yawfails[0] = fails[0] | fails[1];
  yawfails[1] = fails[2] | fails[3];

  // Correct offset
  double measures[6];
  for(int i = 0; i < 6; i++){
      // measures are in cm 
      measures[i] = double(msg->measures[i])/100.0;
  }

  // Get pitch yaw roll from the PixHawk and create a quaternion with no rotation
  tf::Matrix3x3 m(q_imu);
  m.getRPY(roll, pitch, yaw);
  tf::Quaternion q_zero = tf::createQuaternionFromRPY(roll, pitch, 0);

  // Get yaw, use test_node to get the right convention (depending on your laser configuration)
  tf::Vector3 targetx1 = lasers[0].project(measures[0], q_zero);
  tf::Vector3 targetx2 = lasers[1].project(measures[1], q_zero);
  double yaw_x = getYawFromTargets(targetx1, targetx2, 0, 1);

  tf::Vector3 targety1 = lasers[2].project(measures[2], q_zero);
  tf::Vector3 targety2 = lasers[3].project(measures[3], q_zero);
  double yaw_y = -getYawFromTargets(targety2, targety1, 1, 0);

  double yaw_final =  (yaw_x+yaw_y)/2;
  if(yawfails[0] & yawfails[1]){
    yaw_final = 0.0;
  } else if (yawfails[0]) {
    yaw_final = yaw_y;
  } else if (yawfails[1]) {
    yaw_final = yaw_x;
  }

  // Get position
  tf::Quaternion q_correct = tf::createQuaternionFromRPY(roll, pitch,yaw_final);
  tf::Vector3 targets[6];
  for(int i = 0; i < 6; i ++){
    targets[i] = lasers[i].project(measures[i], q_correct);
  }

  // publish
  UAVPose.header.seq = sequence_count;
  UAVPose.header.stamp = ros::Time::now();
  tf::quaternionTFToMsg(q_correct, UAVPose.pose.orientation);

  lastPosition[lastPositionIndex].setX(-(targets[0].x() + targets[1].x())/2.0);
  if(fails[0] & fails[1]){
    lastPosition[lastPositionIndex].setX(lastPosition[(lastPositionIndex-1 >= 0)?lastPositionIndex-1 : 0].x());
  } else if (fails[0]) {
    lastPosition[lastPositionIndex].setX(-targets[1].x());
  } else if (fails[1]) {
    lastPosition[lastPositionIndex].setX(-targets[0].x());
  }

  lastPosition[lastPositionIndex].setY(-(targets[2].y() + targets[3].y())/2.0);
  if(fails[2] & fails[3]){
    lastPosition[lastPositionIndex].setY(lastPosition[(lastPositionIndex-1 >= 0)?lastPositionIndex-1 : 0].y());
  } else if (fails[2]) {
    lastPosition[lastPositionIndex].setY(-targets[3].y());
  } else if (fails[3]) {
    lastPosition[lastPositionIndex].setY(-targets[2].y());
  }


  lastPosition[lastPositionIndex].setZ(-(targets[4].z() + targets[5].z())/2.0);
  if(fails[4] & fails[5]){
    lastPosition[lastPositionIndex].setZ(lastPosition[(lastPositionIndex-1 >= 0)?lastPositionIndex-1 : 0].z());
  } else if (fails[4]) {
    lastPosition[lastPositionIndex].setZ(-targets[5].z());
  } else if (fails[5]) {
    lastPosition[lastPositionIndex].setZ(-targets[4].z());
  }

  UAVPose.pose.position.x = lastPosition[lastPositionIndex].x();
  UAVPose.pose.position.y = lastPosition[lastPositionIndex].y();
  UAVPose.pose.position.z = lastPosition[lastPositionIndex].z();

  lastPositionIndex = (lastPositionIndex+1)%3;

  pose_publisher.publish(UAVPose);
  sequence_count += 1;
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
        offset = double(offsetsList[i]);
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

  // Initialization
  sequence_count = 0;

  lowf_sequence = 0;
  UAVPose.header.frame_id = "/lasers_4D";

  // Reconfigure laser values (from ROS parameters) before using them.
  reconfigure_lasers();

  std::string raw_laser_topic, position_pub_topic, imu_topic;
  int lowrate;
  ros::param::param<int>("low_frequency_rate", lowrate, 8);
  ros::param::param<std::string>("laser_raw_topic", raw_laser_topic, "/flyingros/lasers/raw");
  ros::param::param<std::string>("laser_pose_topic", position_pub_topic, "/flyingros/lasers/pose");
  ros::param::param<std::string>("imu_topic", imu_topic, "/mavros/imu/data");

  ros::Subscriber raw_laser_sub = nh.subscribe(raw_laser_topic, 1, callback_laser_raw);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1, callback_imu);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 1);
  lowFrequency_publisher = nh.advertise<geometry_msgs::PoseStamped>("/flyingros/lasers/pose2mocap", 1);

  ros::Duration lowfrequencyduration(1.0/double(lowrate));
  ros::Time last = ros::Time::now();
  while(ros::ok()){
    if(ros::Time::now() - last > lowfrequencyduration){
      last = ros::Time::now();
      lowFrequency_pub();
    }

    ros::spinOnce();
  }
  return 0;
}
