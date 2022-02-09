#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ck_utilities/Motor.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <thread>
#include <string>
#include <mutex>

#include <action_helper/action_helper.hpp>

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Motor_Status.h>
#include <hmi_agent_node/HMI_Signals.h>

ros::NodeHandle* node;
rio_control_node::Joystick_Status joystick_status;
tf2_ros::TransformBroadcaster * tfBroadcaster;
ActionHelper* action_helper;

#define INCHES_TO_METERS 0.0254

uint32_t config_i = 0;

double turret_target = 0;

Motor * Turret_Shooter_Master;
Motor * Turret_Shooter_Slave_Motor;
Motor * Turret_Yaw_Motor;
Motor * Turret_Hood_Motor;
Motor * Inner_Intake_Motor;

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    (void) msg;


    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, msg.turret_aim_degrees / 360.0, 0);
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, msg.turret_hood_degrees / 360.0, 0);
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, msg.turret_speed_rpm, 0);
}

void config_motors()
{
    Turret_Shooter_Master = new Motor(13, Motor::Motor_Type::TALON_FX);
    Turret_Shooter_Slave_Motor = new Motor(14, Motor::Motor_Type::TALON_FX);
    Turret_Yaw_Motor = new Motor(8, Motor::Motor_Type::TALON_FX);
    Inner_Intake_Motor = new Motor(9, Motor::Motor_Type::TALON_FX);
    Turret_Hood_Motor = new Motor(12, Motor::Motor_Type::TALON_FX);

    Turret_Yaw_Motor->config().set_kP(0.67);
    Turret_Yaw_Motor->config().set_kI(0.0);
    Turret_Yaw_Motor->config().set_kD(0.92);
    Turret_Yaw_Motor->config().set_kF(0.047651);
    Turret_Yaw_Motor->config().set_motion_cruise_velocity(16000);
    Turret_Yaw_Motor->config().set_motion_acceleration(36000);
    Turret_Yaw_Motor->config().set_motion_s_curve_strength(5);
    Turret_Yaw_Motor->config().apply();

    Turret_Hood_Motor->config().set_kP(0.67);
    Turret_Hood_Motor->config().set_kI(0.0);
    Turret_Hood_Motor->config().set_kD(0.92);
    Turret_Hood_Motor->config().set_kF(0.047651);
    Turret_Hood_Motor->config().set_motion_cruise_velocity(16000);
    Turret_Hood_Motor->config().set_motion_acceleration(32000);
    Turret_Hood_Motor->config().set_motion_s_curve_strength(5);
    Turret_Hood_Motor->config().apply();

    Turret_Shooter_Slave_Motor->config().set_follower(true, 13);
    Turret_Shooter_Slave_Motor->config().set_inverted(true);
    Turret_Shooter_Slave_Motor->config().apply();

    Turret_Shooter_Master->config().set_kP(0.03);
    Turret_Shooter_Master->config().set_kI(0.0);
    Turret_Shooter_Master->config().set_kD(0.04);
    Turret_Shooter_Master->config().set_kF(0.047651);
    Turret_Shooter_Master->config().set_closed_loop_ramp(2.5);
    Turret_Shooter_Master->config().set_peak_output_reverse(0.3);
    Turret_Shooter_Master->config().apply();
}

void motor_status_callback(const rio_control_node::Motor_Status& msg)
{
    double motor_rotations = 0;
    bool found_motor = false;

    for(std::vector<rio_control_node::Motor_Info>::const_iterator i = msg.motors.begin();
        i != msg.motors.end();
        i++)
    {
        if((*i).id == 8)
        {
            found_motor = true;
            motor_rotations = (*i).sensor_position;
        }
    }

    if(found_motor)
    {
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "turret_link";

        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 14 * INCHES_TO_METERS;

        tf2::Quaternion q;
        q.setRPY(0, 0, motor_rotations * 2.0 * M_PI);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tfBroadcaster->sendTransform(transformStamped);
    }
}

void handle_shoot_action()
{
    static bool shooting = false;
    static ros::Time shooting_start_time = ros::Time::now();

    if( action_helper->check_action( "Shoot" ) && shooting == false)
    {
        shooting = true;
        shooting_start_time = ros::Time::now();
        action_helper->update_action( "Shoot",
                                        ActionHelper::ACTION_STATUS::COMPLETE );
    }
    if( shooting == true)
    {
        if (ros::Time::now() - shooting_start_time > ros::Duration(1))
        {
            ROS_INFO("SHOOTING!!!");
            shooting = false;
        }
    }
}

void handle_actions()
{
    handle_shoot_action();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_node");
	ros::NodeHandle n;
	node = &n;

    config_motors();

    ros::Subscriber hmi_subscribe = node->subscribe("/HMISignals", 20, hmi_signal_callback);
    ros::Subscriber motor_status_subscribe = node->subscribe("/MotorStatus", 20, motor_status_callback);

    action_helper = new ActionHelper(node);

	tfBroadcaster = new tf2_ros::TransformBroadcaster();

    while(ros::ok())
    {
        handle_actions();

        ros::spinOnce();
    }

	return 0;
}
