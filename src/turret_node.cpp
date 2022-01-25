#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Joystick_Status.h>

ros::NodeHandle* node;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_node");

	ros::NodeHandle n;

	node = &n;

	ros::spin();
	return 0;
}
