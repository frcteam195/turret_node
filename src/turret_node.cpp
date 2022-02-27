#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ck_utilities/Motor.hpp"
#include "ck_utilities/InterpolatingMap.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "hmi_agent/ActionNames.hpp"
#include "math.h"
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
tf2_ros::TransformListener * tfListener;
tf2_ros::Buffer tfBuffer;
ActionHelper* action_helper;

enum class TurretStates
{
    MANUAL,
    TRACKING,
    AIM,
    SHOOT,
};
static TurretStates turret_state = TurretStates::TRACKING;



#define INCHES_TO_METERS 0.0254

uint32_t config_i = 0;

double turret_target = 0;

Motor * Turret_Shooter_Master;
Motor * Turret_Shooter_Slave_Motor;
Motor * Turret_Yaw_Motor;
Motor * Turret_Hood_Motor;
Motor * Inner_Intake_Motor;

float get_angle_to_hub()
{
    tf2::Stamped<tf2::Transform> robot_base_to_hub;
   
                        
    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "hub_link", ros::Time(0)), robot_base_to_hub);
        float theta;
        float x = robot_base_to_hub.getOrigin().getX();
        float y = robot_base_to_hub.getOrigin().getY();
        theta = asin(y/sqrt(x * x + y * y));
        return theta;
        

    }

    catch (...)
    {
        ROS_WARN("Hub transform failed");
    }
    return 0;
}

float get_distance_to_hub()
{
    tf2::Stamped<tf2::Transform> robot_base_to_hub;
   
                        
    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "hub_link", ros::Time(0)), robot_base_to_hub);
        return robot_base_to_hub.getOrigin().length();

    }

    catch (...)
    {
        ROS_WARN("Hub transform failed");
    }
    return 0;

}

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    (void) msg;

    if (msg.shoot_turret)
    {
        Inner_Intake_Motor->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
    }
    else
    {
        Inner_Intake_Motor->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
    }


    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, msg.turret_aim_degrees / 360.0, 0);
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, msg.turret_hood_degrees / 360.0, 0);
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, msg.turret_speed_rpm, 0);
}


   
void set_hood_distance(float distance)
{
    static InterpolatingMap<float, float> hood_lookup_table;
    static bool first_time = true;
    if (first_time)
    {
        hood_lookup_table.insert(0, 100);
        hood_lookup_table.insert(1, 200);
        hood_lookup_table.insert(3, 300);
        hood_lookup_table.insert(4, 800);
        hood_lookup_table.insert(5, 1200);
        hood_lookup_table.insert(10, 0);
        first_time = false;
    }
    float hood_angle = hood_lookup_table.lookup(distance);
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, hood_angle/360.0, 0);

}



void set_turret_angle(float angle)
{
    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, angle/360.0, 0);
}



void step_state_machine() 
{
    switch (turret_state)
    {
        case TurretStates::MANUAL:
        {
            break;

            //use operator controls to aim turret
            
        }
        case TurretStates::TRACKING:
        {
            float distance = get_distance_to_hub();
            float angle = get_angle_to_hub();
            set_turret_angle(angle);
            set_hood_distance(distance);
            break;
           
            //aim turret
            
            //adjust hood
        }
        case TurretStates::AIM:
        {
            break;
            //turn on limelight

            //use limelight to aim

            //set hood based on limelight

            //spin up wheel
        }
        case TurretStates::SHOOT:
        {
            break;

            //shoot the ball
        }
    }
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

    

    
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

	return 0;
}
