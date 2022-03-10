#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ck_utilities/Motor.hpp"
#include "ck_utilities/InterpolatingMap.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight_Control.h"
#include "hmi_agent/ActionNames.hpp"
#include "math.h"
#include "intake_node/Intake_Control.h"
#include "intake_node/Intake_Status.h"
#include <thread>
#include <string>
#include <mutex>

#include <action_helper/action_helper.hpp>

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Motor_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <geometry_msgs/TransformStamped.h>

#define TURRET_SHOOTER_MASTER_CAN_ID 16
#define TURRET_SHOOTER_SLAVE_CAN_ID 17
#define TURRET_YAW_CAN_ID 18
#define TURRET_HOOD_CAN_ID 19

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
    TARGET_LOCKED,
    SPIN_UP_SHOOTER,
    SHOOT,
};

static TurretStates turret_state = TurretStates::TRACKING;
static TurretStates next_turret_state = TurretStates::TRACKING;


#define INCHES_TO_METERS 0.0254

uint32_t config_i = 0;

double turret_target = 0;

Motor * Turret_Shooter_Master;
Motor * Turret_Shooter_Slave_Motor;
Motor * Turret_Yaw_Motor;
Motor * Turret_Hood_Motor;

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

float get_angle_to_hub_limelight()
{
    tf2::Stamped<tf2::Transform> limelight_link_hub;
   
                        
    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "limelight_link_hub", ros::Time(0)), limelight_link_hub);
        float theta;
        float x = limelight_link_hub.getOrigin().getX();
        float y = limelight_link_hub.getOrigin().getY();
        theta = asin(y/sqrt(x * x + y * y));
        return theta;
        

    }

    catch (...)
    {
        ROS_WARN("Hub transform failed");
    }
    return 0;
}

float get_distance_to_hub_limelight()
{
    tf2::Stamped<tf2::Transform> limelight_link_hub;
   
                        
    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "limelight_link_hub", ros::Time(0)), limelight_link_hub);
        return sqrt(pow(limelight_link_hub.getOrigin().getX() , 2) + pow(limelight_link_hub.getOrigin().getY() , 2));
    }

    catch (...)
    {
        ROS_WARN("Hub transform failed");
    }
    return 0;

}

void turn_limelight_on()
{
    static ros::Publisher limelight_control_pub = node->advertise<limelight_vision_node::Limelight_Control>("/LimelightControl", 5);

	limelight_vision_node::Limelight limelight;
	limelight.name = "limelight";
	limelight.pipeline = 1;

    limelight_vision_node::Limelight_Control limelight_control;
    limelight_control.limelights.push_back(limelight);

    limelight_control_pub.publish(limelight_control);
}


void turn_limelight_off()
{
    static ros::Publisher limelight_control_pub = node->advertise<limelight_vision_node::Limelight_Control>("/LimelightControl", 5);

	limelight_vision_node::Limelight limelight;
	limelight.name = "limelight";
	limelight.pipeline = 0;

    limelight_vision_node::Limelight_Control limelight_control;
    limelight_control.limelights.push_back(limelight);

    limelight_control_pub.publish(limelight_control);
}

static bool readyToShoot = false;

void intake_status_callback(const intake_node::Intake_Status& msg)
{
    (void) msg;

    readyToShoot = msg.readyToShoot;
}

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    (void) msg;


    //still needs to be updated
}

static bool limelightHasTarget = false;

void limelight_status_callback(const limelight_vision_node::Limelight_Status& msg)
{
    (void) msg;

    limelightHasTarget = msg.limelights[0].target_valid;
    
}

static float actualShooterRPM = 0;

bool reached_target_vel(float targetVel)
{
    
    return actualShooterRPM <= targetVel + 25 && actualShooterRPM >= targetVel - 25;
    
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

float shooter_rpm;

void set_shooter_vel(float distance)
{
    static InterpolatingMap<float, float> shooter_rpm_lookup_table;
    static bool first_time = true;
    if (first_time)
    {
        shooter_rpm_lookup_table.insert(0, 5000);
        shooter_rpm_lookup_table.insert(40, 5000);
        first_time = false;
    }
    shooter_rpm = shooter_rpm_lookup_table.lookup(distance);
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, shooter_rpm, 0);

}

void turn_shooter_off()
{
    Turret_Shooter_Master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);

}


void set_turret_angle(float angle)
{
    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, angle/360.0, 0);
}



void step_state_machine() 
{
    static ros::Time time_state_entered = ros::Time::now();

	if(turret_state != next_turret_state)
	{
		time_state_entered = ros::Time::now();
	}

	ros::Duration time_in_state = ros::Time::now() - time_state_entered;

    switch (turret_state)
    {
        case TurretStates::MANUAL:
        {

            break;

            //use operator controls to aim turret
            
        }
        case TurretStates::TRACKING:
        {
            turn_limelight_off();
            float distance = get_distance_to_hub();
            float angle = get_angle_to_hub();
            set_turret_angle(angle);
            set_hood_distance(distance);
            turn_shooter_off();

            break;
           
            //aim turret
            
            //adjust hood
        }
        case TurretStates::AIM:
        {
            turn_limelight_on();
            
            float distance = get_distance_to_hub();
            float angle = get_angle_to_hub();
            set_turret_angle(angle);
            set_hood_distance(distance);
            turn_shooter_off();

            break;
            
            //turn on limelight

            //use limelight to aim

            //set hood based on limelight

            //spin up wheel
        }
        case TurretStates::TARGET_LOCKED:
        {
            turn_limelight_on();
            
            float distance = get_distance_to_hub_limelight();
            float angle = get_angle_to_hub_limelight();
            set_turret_angle(angle);
            set_hood_distance(distance);
            turn_shooter_off();

            break;

            
        }
        case TurretStates::SPIN_UP_SHOOTER:
        {
            
            turn_limelight_on();
            
            float distance = get_distance_to_hub_limelight();
            float angle = get_angle_to_hub_limelight();
            set_turret_angle(angle);
            set_hood_distance(distance);
            set_shooter_vel(distance);

            break;

            //sets shooter vel
        }
        case TurretStates::SHOOT:
        {
            
            turn_limelight_on();
            
            float distance = get_distance_to_hub_limelight();
            float angle = get_angle_to_hub_limelight();
            set_turret_angle(angle);
            set_hood_distance(distance);
            set_shooter_vel(distance);
            static ros::Publisher intakeControlPublisher = node->advertise<intake_node::Intake_Control>("/IntakeControl" , 1);
            intake_node::Intake_Control controlMsg;
            controlMsg.command_shoot = true;
            intakeControlPublisher.publish(controlMsg);

              

            break;

            //shoot the ball
        }
    }



    switch (turret_state)
    {
        case TurretStates::MANUAL:
        {

            break;

            //use operator controls to aim turret
            
        }
        case TurretStates::TRACKING:
        {
            if ( readyToShoot )
            {
                turret_state = TurretStates::AIM;
            }
            break;
           
            //aim turret
            
            //adjust hood
        }
        case TurretStates::AIM:
        {
            if ( limelightHasTarget )
            {
                turret_state = TurretStates::TARGET_LOCKED;
            }
            break;
            
            //turn on limelight

            //use limelight to aim

            //set hood based on limelight

            //spin up wheel
        }
        case TurretStates::TARGET_LOCKED:
        {
            turret_state = TurretStates::TARGET_LOCKED;

            break;

        }
        case TurretStates::SPIN_UP_SHOOTER:
        {
            if (reached_target_vel(shooter_rpm) == true)
            {
                turret_state = TurretStates::SHOOT;
            }
           
            break;

            //sets shooter vel
        }
        case TurretStates::SHOOT:
        {
            if (time_in_state > ros::Duration(1))
			{
				turret_state = TurretStates::TRACKING;
			}

            break;

            //shoot the ball
        }
    }
        ROS_INFO("Turret state: %d", (int) turret_state);
}

void config_motors()
{
    Turret_Shooter_Master = new Motor(TURRET_SHOOTER_MASTER_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Shooter_Slave_Motor = new Motor(TURRET_SHOOTER_SLAVE_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Yaw_Motor = new Motor(TURRET_YAW_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Hood_Motor = new Motor(TURRET_HOOD_CAN_ID, Motor::Motor_Type::TALON_FX);

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

    Turret_Shooter_Slave_Motor->config().set_follower(true, TURRET_SHOOTER_MASTER_CAN_ID);
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
    (void) msg;
    // double motor_rotations = 0;
    // bool found_motor = false;

    for(std::vector<rio_control_node::Motor_Info>::const_iterator i = msg.motors.begin();
        i != msg.motors.end();
        i++)
    {
        if((*i).id == TURRET_SHOOTER_MASTER_CAN_ID)
        {
            actualShooterRPM = (*i).sensor_velocity;
        }
    }

    // if(found_motor)
    // {
    //     geometry_msgs::TransformStamped transformStamped;

    //     transformStamped.header.stamp = ros::Time::now();
    //     transformStamped.header.frame_id = "base_link";
    //     transformStamped.child_frame_id = "turret_link";

    //     transformStamped.transform.translation.x = 0;
    //     transformStamped.transform.translation.y = 0;
    //     transformStamped.transform.translation.z = 14 * INCHES_TO_METERS;

    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, motor_rotations * 2.0 * M_PI);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     tfBroadcaster->sendTransform(transformStamped);
    // }
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_node");
	ros::NodeHandle n;
	node = &n;

    config_motors();

    ros::Subscriber hmi_subscribe = node->subscribe("/HMISignals", 1, hmi_signal_callback);
    ros::Subscriber motor_status_subscribe = node->subscribe("/MotorStatus", 1, motor_status_callback);
    ros::Subscriber limelight_status_subscribe = node->subscribe("/LimelightStatus", 1, limelight_status_callback);
    ros::Subscriber intake_status_subscribe = node->subscribe("/IntakeStatus", 1, intake_status_callback);

    action_helper = new ActionHelper(node);

	tfBroadcaster = new tf2_ros::TransformBroadcaster();

    

    
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();

        step_state_machine();

        rate.sleep();
    }

	return 0;
}
