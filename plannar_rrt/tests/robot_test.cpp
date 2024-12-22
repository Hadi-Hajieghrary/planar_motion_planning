#include <gtest/gtest.h>
#include "robot.hpp"

// Test for Robot Constructor
TEST(RobotTest, ConstructorInitializesCorrectly) {
    BoundedValue theta1(-M_PI, M_PI, 0.0);
    BoundedValue theta2(-M_PI, M_PI, 0.0);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(1.0, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    auto joint_values = robot.getJointValues();

    EXPECT_DOUBLE_EQ(joint_values[0].getValue(), 0.0);
    EXPECT_DOUBLE_EQ(joint_values[1].getValue(), 0.0);
    EXPECT_DOUBLE_EQ(joint_values[2].getValue(), 0.0);
}

// Test for Joint Value Update
TEST(RobotTest, SetJointValues) {
    BoundedValue theta1(-M_PI, M_PI, 0.0);
    BoundedValue theta2(-M_PI, M_PI, 0.0);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(1.0, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    robot.setJointValues(0.5, -0.5, 1.0);
    auto joint_values = robot.getJointValues();

    EXPECT_DOUBLE_EQ(joint_values[0].getValue(), 0.5);
    EXPECT_DOUBLE_EQ(joint_values[1].getValue(), -0.5);
    EXPECT_DOUBLE_EQ(joint_values[2].getValue(), 1.0);
}

// Test for Forward Kinematics
TEST(RobotTest, ForwardKinematics) {
    BoundedValue theta1(-M_PI, M_PI, M_PI / 4);
    BoundedValue theta2(-M_PI, M_PI, M_PI / 4);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(0.3, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    auto ee_pose = robot.getEEPose();

    // Verify end-effector pose
    EXPECT_NEAR(ee_pose.position.x, 1.0 * cos(theta1.getValue())  + 1.5 , 1e-3);
    EXPECT_NEAR(ee_pose.position.y, 0.3 + 1.0 * sin(theta1.getValue()), 1e-3);
    EXPECT_NEAR(ee_pose.orientation, 0.0 , 1e-3);
}

// Test for Inverse Kinematics
TEST(RobotTest, InverseKinematics) {
    BoundedValue theta1(-M_PI, M_PI, M_PI / 4);
    BoundedValue theta2(-M_PI, M_PI, M_PI / 4);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(0.3, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    auto ee_pose = robot.getEEPose();

    Pose2D target_pose = ee_pose;
    std::array<double, 3> joint_values;
    bool success = robot.inverseKinematics(target_pose, joint_values);

    EXPECT_TRUE(success);
    EXPECT_NEAR(joint_values[0], theta1.getValue(), 1e-3);
    EXPECT_NEAR(joint_values[1], theta2.getValue(), 1e-3);
    EXPECT_NEAR(joint_values[2], theta3.getValue(), 1e-3);
}

// Test for Collision Objects
TEST(RobotTest, GetCollisionObjects) {
    BoundedValue theta1(-M_PI, M_PI, 0.0);
    BoundedValue theta2(-M_PI, M_PI, 0.0);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(1.0, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    std::vector<Box> collision_objects;
    robot.getRobotCollisionObjects(collision_objects);

    EXPECT_EQ(collision_objects.size(), 4); // Expect 4 links to have collision objects
}

// Test for Collision Objects with Grasped Object
TEST(RobotTest, GetCollisionObjectsWithGraspedObject) {
    BoundedValue theta1(-M_PI, M_PI, 0.0);
    BoundedValue theta2(-M_PI, M_PI, 0.0);
    BoundedValue theta3(-M_PI, M_PI, 0.0);

    Robot robot(1.0, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    Box grasped_object({{2.0, 2.0}, 0.0}, 0.5, 0.5);
    std::vector<Box> collision_objects;
    robot.getRobotCollisionObjects(grasped_object, collision_objects);

    EXPECT_EQ(collision_objects.size(), 5); // Expect 4 links + 1 grasped object
}


