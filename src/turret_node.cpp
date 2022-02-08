#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ck_utilities/Motor.hpp"

#include <thread>
#include <string>
#include <mutex>

#include <rio_control_node/Joystick_Status.h>
#include <hmi_agent_node/HMI_Signals.h>

ros::NodeHandle* node;
rio_control_node::Joystick_Status joystick_status;

uint32_t config_i = 0;

double turret_target = 0;

Motor * Turret_Shooter_Master;
Motor * Turret_Shooter_Slave_Motor;
Motor * Turret_Yaw_Motor;
Motor * Turret_Hood_Motor;

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    (void) msg;
    ROS_INFO("GOT INTO HERE EVEN!");

    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, 0, 0);
}

void config_motors()
{
    Turret_Shooter_Master = new Motor(13, Motor::Motor_Type::TALON_FX);
    Turret_Shooter_Slave_Motor = new Motor(14, Motor::Motor_Type::TALON_FX);
    Turret_Yaw_Motor = new Motor(8, Motor::Motor_Type::TALON_FX);
    Turret_Hood_Motor = new Motor(12, Motor::Motor_Type::TALON_FX);

    Turret_Yaw_Motor->config().set_kP(0.67);
    Turret_Yaw_Motor->config().set_kI(0.0);
    Turret_Yaw_Motor->config().set_kD(0.92);
    Turret_Yaw_Motor->config().set_kF(0.047651);
    Turret_Yaw_Motor->config().set_motion_cruise_velocity(16000);
    Turret_Yaw_Motor->config().set_motion_acceleration(36000);
    Turret_Yaw_Motor->config().set_motion_s_curve_strength(5);

    Turret_Hood_Motor->config().set_kP(0.67);
    Turret_Hood_Motor->config().set_kI(0.0);
    Turret_Hood_Motor->config().set_kD(0.92);
    Turret_Hood_Motor->config().set_kF(0.047651);
    Turret_Hood_Motor->config().set_motion_cruise_velocity(16000);
    Turret_Hood_Motor->config().set_motion_acceleration(32000);
    Turret_Hood_Motor->config().set_motion_s_curve_strength(5);
    
    Turret_Shooter_Slave_Motor->config().set_follower(true, 13);
    Turret_Shooter_Slave_Motor->config().set_inverted(true);

    Turret_Shooter_Master->config().set_kP(0.03);
    Turret_Shooter_Master->config().set_kI(0.0);
    Turret_Shooter_Master->config().set_kD(0.04);
    Turret_Shooter_Master->config().set_kF(0.047651);
    Turret_Shooter_Master->config().set_closed_loop_ramp(2.5);
    Turret_Shooter_Master->config().set_peak_output_reverse(0.3);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_node");
	ros::NodeHandle n;
	node = &n;

    config_motors();

    ros::Subscriber hmi_subscribe = node->subscribe("/HMI_Signals", 20, hmi_signal_callback);

	ros::spin();
	return 0;
}
