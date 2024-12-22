#include <gtest/gtest.h>
#include "utility.hpp"
#include <cmath>

// Test for BoundedValue class
TEST(BoundedValueTest, InitializationAndClamping) {
    BoundedValue bv(-10.0, 10.0, 5.0);
    EXPECT_DOUBLE_EQ(bv.getValue(), 5.0);

    bv.setValue(20.0);
    EXPECT_DOUBLE_EQ(bv.getValue(), 10.0); // Should clamp to max

    bv.setValue(-20.0);
    EXPECT_DOUBLE_EQ(bv.getValue(), -10.0); // Should clamp to min
}

TEST(BoundedValueTest, ArithmeticOperations) {
    BoundedValue bv1(-10.0, 10.0, 5.0);
    BoundedValue bv2(-10.0, 10.0, -3.0);

    auto result = bv1 + bv2;
    EXPECT_DOUBLE_EQ(result.getValue(), 2.0);

    result = bv1 - bv2;
    EXPECT_DOUBLE_EQ(result.getValue(), 8.0);

    result = bv1 * bv2;
    EXPECT_DOUBLE_EQ(result.getValue(), -10.0);

    result = bv1 / bv2;
    EXPECT_DOUBLE_EQ(result.getValue(), -5.0 / 3.0);
}

TEST(BoundedValueTest, RandomValue) {
    BoundedValue bv(0.0, 1.0, 0.5);
    double randomValue = bv.getRandomValue();
    EXPECT_TRUE(randomValue >= 0.0 && randomValue <= 1.0);
}

// Test for Box class
TEST(BoxTest, CornerCalculations) {
    Pose2D pose{{0.0, 0.0}, 0.0}; // No rotation
    Box box(pose, 4.0, 2.0);

    auto corners = box.getCorners();
    ASSERT_EQ(corners.size(), 4);

    EXPECT_DOUBLE_EQ(corners[0].x, 0.0);
    EXPECT_DOUBLE_EQ(corners[0].y, 1.0);

    EXPECT_DOUBLE_EQ(corners[1].x, 4.0);
    EXPECT_DOUBLE_EQ(corners[1].y, 1.0);

    EXPECT_DOUBLE_EQ(corners[2].x, 4.0);
    EXPECT_DOUBLE_EQ(corners[2].y, -1.0);

    EXPECT_DOUBLE_EQ(corners[3].x, 0.0);
    EXPECT_DOUBLE_EQ(corners[3].y, -1.0);
}

TEST(BoxTest, RotatedCorners) {
    Pose2D pose{{0.0, 0.0}, M_PI / 4.0}; // 45-degree rotation
    Box box(pose, 4.0, 2.0);

    auto corners = box.getCorners();
    ASSERT_EQ(corners.size(), 4);

    // Ensure corners are rotated properly (precision may vary)
    EXPECT_NEAR(corners[0].x, box.getHeight() / 2.0 * cos(pose.orientation + M_PI / 2.0), 0.01);
    EXPECT_NEAR(corners[0].y, box.getHeight() / 2.0 * sin(pose.orientation + M_PI / 2.0), 0.01);

    EXPECT_NEAR(corners[1].x, box.getHeight() / 2.0 * cos(pose.orientation + M_PI / 2.0) + box.getWidth() * cos(pose.orientation), 0.01);
    EXPECT_NEAR(corners[1].y, box.getHeight() / 2.0 * sin(pose.orientation + M_PI / 2.0) + box.getWidth() * sin(pose.orientation), 0.01);

}

// Test for distance functions
TEST(DistanceTest, EuclideanDistance) {
    std::array<double, 3> point1{1.0, 2.0, 0.0};
    std::array<double, 3> point2{4.0, 6.0, 0.0};

    EXPECT_DOUBLE_EQ(distance(point1, point2), 5.0);
}

TEST(DistanceTest, MinDistanceBetweenLineSegments) {
    Point2D p1{0.0, 0.0}, p2{1.0, 0.0};
    Point2D q1{0.0, 1.0}, q2{1.0, 1.0};

    EXPECT_DOUBLE_EQ(minDistanceBetweenLineSegments(p1, p2, q1, q2), 1.0);
}

// Test for Box clearance
TEST(BoxTest, AreBoxesClear) {
    Pose2D pose1{{0.0, 0.0}, 0.0};
    Pose2D pose2{{5.0, 0.0}, 0.0};


    Box box1(pose1, 4.0, 2.0);
    Box box2(pose2, 4.0, 2.0);

    EXPECT_TRUE(areBoxesClear(box1, box2, 0.5));
}
