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
    float output = calculate_turret_angle(-280, -270);
    ASSERT_EQ(output, -280);

    output = calculate_turret_angle(-294, -270);
    ASSERT_EQ(output, 66);

    output = calculate_turret_angle(-285, -270);
    ASSERT_EQ(output, -285);

    output = calculate_turret_angle(70, 60);
    ASSERT_EQ(output, 70);

    output = calculate_turret_angle(100, 60);
    ASSERT_EQ(output, 100);

    output = calculate_turret_angle(120, 60);
    ASSERT_EQ(output, -240);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_turret_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}