#include "test_turret_node.hpp"
#include "turret_node.hpp"
#include "ros/ros.h"

#include <gtest/gtest.h>

TEST(SampleTest, Test_Test)
{
    ASSERT_TRUE(1);
    ASSERT_FALSE(0);
}

TEST(TurretTest, Test_Wrap_Around)
{
    float output = calculate_turret_angle(280, 270);
    ASSERT_EQ(output, -80);

    output = calculate_turret_angle(295, 270);
    ASSERT_EQ(output, -65);

    output = calculate_turret_angle(285, 270);
    ASSERT_EQ(output, -75);

    output = calculate_turret_angle(-70, -60);
    ASSERT_EQ(output, -70);

    output = calculate_turret_angle(-100, -60);
    ASSERT_EQ(output, 260);

    output = calculate_turret_angle(-115, -60);
    ASSERT_EQ(output, 245);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_turret_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}