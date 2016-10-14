#include "tf/transform_datatypes.h"
#include "flyingros_pose/laser_algorithm_functions.h"
#include <gtest/gtest.h>
#include <iostream>
// Bring in gtest

using namespace flyingros_pose;
//

void inline isNearVector3(tf::Vector3 actual, tf::Vector3 wanted){
    EXPECT_NEAR(actual.x(), wanted.x(), 0.01);
    EXPECT_NEAR(actual.y(), wanted.y(), 0.01);
    EXPECT_NEAR(actual.z(), wanted.z(), 0.01);
}

// MODE : SRPY,
// Static axis (rotation axis DO NOT rotate with the system by default with createQuaternionFromRPY)
// Roll CW, Pitch CW, Yaw CCW
TEST(TFQuaternion, createQuaternionFromRPY) {
    tf::Vector3 laser1(2,3,2);
    tf::Vector3 laser2(-1,-1,-1);
    tf::Vector3 result;
    tf::Quaternion q = tf::createQuaternionFromRPY(deg2radf(10),0,0);
    result = tf::quatRotate(q, laser1);
    isNearVector3(result, tf::Vector3(2,2.607,2.491));
    result = tf::quatRotate(q, laser2);
    isNearVector3(result, tf::Vector3(-1,-0.811,-1.158));
    q = tf::createQuaternionFromRPY(deg2radf(10),deg2radf(20),0);
    result = tf::quatRotate(q, laser1);
    isNearVector3(result, tf::Vector3(2.731,2.607,1.656));
    result = tf::quatRotate(q, laser2);
    isNearVector3(result, tf::Vector3(-1.336,-0.811,-0.747));
    q = tf::createQuaternionFromRPY(deg2radf(10),deg2radf(20),deg2radf(35));
    result = tf::quatRotate(q, laser1);
    isNearVector3(result, tf::Vector3(0.742,3.702,1.656));
    result = tf::quatRotate(q, laser2);
    isNearVector3(result, tf::Vector3(-0.629,-1.431,-0.747));
    q = tf::createQuaternionFromRPY(deg2radf(-8),deg2radf(-20),deg2radf(-8));
    result = tf::quatRotate(q, laser1);
    isNearVector3(result, tf::Vector3(1.784,3.03,2.153));
    result = tf::quatRotate(q, laser2);
    isNearVector3(result, tf::Vector3(-0.799,-1.028,-1.142));
}

// It shows it works
TEST(FlyingrosPose, NullingYaw)
{
    tf::Quaternion q_yaw = tf::createQuaternionFromRPY(deg2radf(-8),deg2radf(-20),deg2radf(-8));
    tf::Quaternion q_no_yaw_reference = tf::createQuaternionFromRPY(deg2radf(-8),deg2radf(-20),0);
    tf::Quaternion q_no_yaw_computed = nullYawQuaternion(q_yaw);

    EXPECT_NEAR(q_no_yaw_reference.x(), q_no_yaw_computed.x(), 0.01);
    EXPECT_NEAR(q_no_yaw_reference.y(), q_no_yaw_computed.y(), 0.01);
    EXPECT_NEAR(q_no_yaw_reference.z(), q_no_yaw_computed.z(), 0.01);
    EXPECT_NEAR(q_no_yaw_reference.w(), q_no_yaw_computed.w(), 0.01);
}

// CAUTION PLACING LASERS.
// Position of the laser is positive if nearer of the wall
// Position of the laser is negative if further of the wall
// The target X element if pointing the wall in X is the X coordinate of the multicopter
TEST(FlyingrosPose, LaserProjection)
{
    // RPY = (15,-20,-7)
    // Measure1 = 9.396m
    // Target1 = (10,1.17,6.443)
    // Measure2 = 11.368m
    // Target2 = (10,-1.94,2.395)
    Laser laser1(tf::Vector3(2,3,2), tf::Vector3(1,0,0), 0);
    Laser laser2(tf::Vector3(-1,-1,-1), tf::Vector3(1,0,0), 0);

    tf::Quaternion q = tf::createQuaternionFromRPY(deg2radf(15),deg2radf(-20),deg2radf(-7));

    tf::Vector3 target = laser1.project(9.396, q);
    isNearVector3(target, tf::Vector3(10,1.17,6.443));
    target = laser2.project(11.368, q);
    isNearVector3(target, tf::Vector3(10,-1.94,2.395));
}

// Verify we have the right Yaw from the targets only
/// TEST NOT WORKING YET
TEST(FlyingrosPose, LaserYaw) {
    // RPY = (15,-20,-7)
    // Measure1 = 9.396m
    // Target1 = (10,1.17,6.443)
    // Measure2 = 11.368m
    // Target2 = (10,-1.94,2.395)
    Laser laser1(tf::Vector3(2,3,2), tf::Vector3(1,0,0), 0);
    Laser laser2(tf::Vector3(-1,-1,-1), tf::Vector3(1,0,0), 0);
    tf::Quaternion q = tf::createQuaternionFromRPY(deg2radf(15),deg2radf(-20),deg2radf(-7));
    tf::Quaternion q_no_yaw = nullYawQuaternion(q);
    tf::Vector3 target1 = laser1.project(9.396, q_no_yaw);
    tf::Vector3 target2 = laser2.project(11.368, q_no_yaw);

    // The actual test

    double yaw = getYawFromTargets(target1, target2, 0, 1);

    EXPECT_NEAR(yaw, deg2radf(-7), 0.001);
    //tf::Quaternion q = tf::createQuaternionFromRPY(deg2radf(15),deg2radf(-20),deg2radf(-7));
    EXPECT_EQ(1,1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
