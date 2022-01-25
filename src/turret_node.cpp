#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <rio_control_node/Motor_Configuration.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Joystick_Status.h>

ros::NodeHandle* node;
rio_control_node::Joystick_Status joystick_status;

ros::Publisher motor_control_pub;
ros::Publisher motor_config_pub;

int config_i = 0;

double turret_target = 0;


void set_turret_clockwise()
{
    turret_target += 0.01;
}

void set_turret_counter_clockwise()
{
    turret_target -= 0.01;
}

void publish_angle( float input_angle )
{
    rio_control_node::Motor_Control motor_control;
    rio_control_node::Motor motor;

    motor.id = 8;
    motor.controller_type = rio_control_node::Motor::TALON_FX;
    motor.control_mode = rio_control_node::Motor::POSITION;
    motor.output_value = input_angle;
    motor.arbitrary_feedforward = 0;

    motor_control.motors.push_back(motor);
    motor_control_pub.publish(motor_control);
}

void publish_config()
{
    rio_control_node::Motor_Configuration all_config;

	rio_control_node::Motor_Config motor_config;
	motor_config.id = 8;
	motor_config.controller_type = (int8_t)rio_control_node::Motor_Config::TALON_FX;
	motor_config.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
	motor_config.invert_type = rio_control_node::Motor_Config::NONE;
	motor_config.neutral_mode = rio_control_node::Motor_Config::COAST;
	motor_config.voltage_compensation_saturation = 12;
	motor_config.voltage_compensation_enabled = true;
	all_config.motors.push_back(motor_config);

    motor_config_pub.publish(all_config);


}

void joystickStatusCallback( const rio_control_node::Joystick_Status& joystick_in )
{
    joystick_status = joystick_in;


    if( joystick_status.joysticks.size() <= 0 )
    {
        return;
    }

    if( joystick_status.joysticks[0].axes.size() <= 0 )
    {
        return;
    }

    // now at this point, we can use the first axes in the first joystick

    float deadband = 0.2;
    float value_axes = joystick_status.joysticks[0].axes[0];

    if( abs(value_axes) < deadband  )
    {
        return;
    }

    if( value_axes > 0 )
    {
        set_turret_clockwise();
    }

    if( value_axes < 0 )
    {
        set_turret_counter_clockwise();
    }

    publish_angle( turret_target );

    if(config_i % 100 == 0)
    {
        publish_config();
    }
    config_i++;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_node");
	ros::NodeHandle n;
	node = &n;

    ros::Subscriber joystick_sub = node->subscribe("JoystickStatus", 10, joystickStatusCallback);
    motor_control_pub = node->advertise<rio_control_node::Motor_Control>("MotorControl", 1);
	motor_config_pub = node->advertise<rio_control_node::Motor_Configuration>("MotorConfiguration", 1);

	ros::spin();
	return 0;
}
