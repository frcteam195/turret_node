#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ck_utilities/Motor.hpp"
#include "ck_utilities/InterpolatingMap.hpp"
#include "ck_utilities/CKMath.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight_Control.h"
#include "hmi_agent/ActionNames.hpp"
#include "math.h"
#include "intake_node/Intake_Control.h"
#include "intake_node/Intake_Status.h"
#include "turret_node/turret_diagnostics.h"
#include "turret_node/turret_status.h"
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

ros::NodeHandle *node;
rio_control_node::Joystick_Status joystick_status;
tf2_ros::TransformBroadcaster *tfBroadcaster;
tf2_ros::TransformListener *tfListener;
tf2_ros::Buffer tfBuffer;
ActionHelper *action_helper;

enum class TurretStates
{
    MANUAL,
    TRACKING,
    AIM,
    TARGET_LOCKED,
    SPIN_UP_SHOOTER,
    SHOOT,
    PREPARE_CLIMB
};

static constexpr float SHOOTER_RPM_DELTA = 250;
static constexpr float HOOD_DEG_DELTA = 2;
static constexpr float TURRET_YAW_DEG_DELTA = 2;
static constexpr float SHOOTER_RPM_FILTER_TIME = 0.2;


static TurretStates turret_state = TurretStates::TRACKING;
static TurretStates next_turret_state = TurretStates::TRACKING;
static bool readyToShoot = false;
static bool limelightHasTarget = false;
static float actualShooterRPM = 0;
static float actualTurretYawDeg = 0;
static float actualHoodDeg = 0;
static float target_shooter_rpm = 0;
static float target_hood_angle = 0;
static float target_yaw_angle = 0;
static float target_manual_shooter_rpm = 0;
static float target_manual_hood_angle = 0;
static float target_manual_yaw_angle = 0;
static bool at_target_shooter_rpm = false;
static bool at_target_hood_angle = false;
static bool at_target_yaw_angle = false;
static bool at_target_limelight_angle = false;
static bool inside_distance_window = false;
static bool spin_up_clearance = false;
static bool shoot_clearance = false;
static bool allowed_to_shoot = false;
static bool manual_control_enabled = false;
static bool about_to_shoot = false;
static float limelight_tx = 0;
static float at_shooter_rpm_time = 0;

static bool hooks_deployed = false;

std::string turret_state_to_string(TurretStates state)
{
    switch (state)
    {
    case TurretStates::MANUAL:
    {
        return "MANUAL";
        break;
    }
    case TurretStates::TRACKING:
    {
        return "TRACKING";
        break;
    }
    case TurretStates::AIM:
    {
        return "AIM";
        break;
    }
    case TurretStates::TARGET_LOCKED:
    {
        return "TARGET_LOCKED";
        break;
    }
    case TurretStates::SPIN_UP_SHOOTER:
    {
        return "SPIN_UP_SHOOTER";
        break;
    }
    case TurretStates::SHOOT:
    {
        return "SHOOT";
        break;
    }
    case TurretStates::PREPARE_CLIMB:
    {
        return "PREPARE_CLIMB";
        break;
    }
    }
    return "INVALID";
}

#define INCHES_TO_METERS 0.0254

Motor *Turret_Shooter_Master;
Motor *Turret_Shooter_Slave_Motor;
Motor *Turret_Yaw_Motor;
Motor *Turret_Hood_Motor;

float get_angle_to_hub()
{
    tf2::Stamped<tf2::Transform> robot_base_to_hub;

    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "hub_link", ros::Time(0)), robot_base_to_hub);
        float theta;
        float x = robot_base_to_hub.getOrigin().getX();
        float y = robot_base_to_hub.getOrigin().getY();
        theta = atan2(y, x);
        return theta * 180.0 / M_PI;
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
        theta = atan2(y, x);
        return theta * 180.0 / M_PI;
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
        return sqrt(pow(limelight_link_hub.getOrigin().getX(), 2) + pow(limelight_link_hub.getOrigin().getY(), 2));
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
    limelight.pipeline = 1;

    limelight_vision_node::Limelight_Control limelight_control;
    limelight_control.limelights.push_back(limelight);

    limelight_control_pub.publish(limelight_control);
}

void intake_status_callback(const intake_node::Intake_Status &msg)
{
    readyToShoot = msg.readyToShoot;
}

void hmi_signal_callback(const hmi_agent_node::HMI_Signals &msg)
{
    target_manual_hood_angle = msg.turret_hood_degrees;
    target_manual_shooter_rpm = msg.turret_speed_rpm;
    target_manual_yaw_angle = msg.turret_aim_degrees;
    manual_control_enabled = msg.turret_manual;
    allowed_to_shoot = msg.allow_shoot;
    if (!hooks_deployed)
    {
        hooks_deployed = msg.deploy_hooks;
    }

    // still needs to be updated
}

void limelight_status_callback(const limelight_vision_node::Limelight_Status &msg)
{
    limelightHasTarget = msg.limelights[0].target_valid;
    limelight_tx = msg.limelights[0].target_dx_deg;
}

static float target_vel_offset = 0;
static float target_hood_offset = 0;
static float target_yaw_offset = 0;
bool reached_target_vel(float targetVel)
{
    target_vel_offset = fabs(targetVel - actualShooterRPM);
    return ck::math::inRange(targetVel - actualShooterRPM, SHOOTER_RPM_DELTA);
}

bool reached_target_hood_deg(float targetHoodDeg)
{
    target_hood_offset = fabs(targetHoodDeg - actualHoodDeg);
    return ck::math::inRange(targetHoodDeg - actualHoodDeg, HOOD_DEG_DELTA);
}

bool reached_target_turret_yaw_deg(float targetTurretYawDeg)
{
    target_yaw_offset = fabs(targetTurretYawDeg - actualTurretYawDeg);
    return ck::math::inRange(targetTurretYawDeg - actualTurretYawDeg, TURRET_YAW_DEG_DELTA);
}

bool reached_limelight_position(float limelightPosition)
{
    return ck::math::inRange(limelightPosition, TURRET_YAW_DEG_DELTA);
}

void set_hood_distance(float distance)
{
    static InterpolatingMap<float, float> hood_lookup_table;
    static bool first_time = true;
    if (first_time)
    {
        hood_lookup_table.insert(2.75, 7.3);
        hood_lookup_table.insert(3.026, 9.82);
        hood_lookup_table.insert(3.88, 12.7);
        hood_lookup_table.insert(5.176, 17.67);
        first_time = false;
    }
    target_hood_angle = hood_lookup_table.lookup(distance);
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, target_hood_angle / 360.0, 0);
}

void set_shooter_vel(float distance)
{
    static InterpolatingMap<float, float> shooter_rpm_lookup_table;
    static bool first_time = true;
    if (first_time)
    {
        shooter_rpm_lookup_table.insert(2.75, 1335);
        shooter_rpm_lookup_table.insert(3.026, 1325);
        // MGT TBD Removing third sample since it's an outlier
        //shooter_rpm_lookup_table.insert(3.88, 1425);
        shooter_rpm_lookup_table.insert(5.176, 1750);
        shooter_rpm_lookup_table.insert(7, 2366); // this is an extrapolated value

        first_time = false;
    }
    target_shooter_rpm = shooter_rpm_lookup_table.lookup(distance);
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, target_shooter_rpm, 0);
}

void turn_shooter_off()
{
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, 0, 0);
}

void set_turret_angle(float angleDeg)
{
    target_yaw_angle = angleDeg;
    target_yaw_angle += 180;
    while(target_yaw_angle > 360.0)
    {
        target_yaw_angle -= 360.0;
    }
    while(target_yaw_angle < 0)
    {
        target_yaw_angle += 360.0
    }
    target_yaw_angle -= 180;
    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, angleDeg / 360.0, 0);
}

void step_state_machine()
{
    static ros::Time time_state_entered = ros::Time::now();
    static ros::Publisher intakeControlPublisher = node->advertise<intake_node::Intake_Control>("/IntakeControl", 1);
    intake_node::Intake_Control controlMsg;
    controlMsg.command_shoot = false;

    if (hooks_deployed)
    {
        next_turret_state = TurretStates::PREPARE_CLIMB;
    }

    if (manual_control_enabled)
    {
        next_turret_state = TurretStates::MANUAL;
    }

    if (turret_state != next_turret_state)
    {
        time_state_entered = ros::Time::now();
    }

    turret_state = next_turret_state;

    ros::Duration time_in_state = ros::Time::now() - time_state_entered;

    float distance = get_distance_to_hub();
    float angle = get_angle_to_hub();

    if(limelightHasTarget)
    {
        distance = get_distance_to_hub_limelight();
        angle = get_angle_to_hub_limelight();
    }

    bool at_shooter_rpm = turret_state == TurretStates::SPIN_UP_SHOOTER && reached_target_vel(target_shooter_rpm);
    static ros::Time shooter_rpm_false_time = ros::Time::now();
    
    if (!at_shooter_rpm)
    {
        shooter_rpm_false_time = ros::Time::now();
    }

    at_shooter_rpm_time = (float) ros::Duration(ros::Time::now() - shooter_rpm_false_time).toSec();

    at_target_shooter_rpm = at_shooter_rpm && at_shooter_rpm_time > SHOOTER_RPM_FILTER_TIME;
    at_target_hood_angle = reached_target_hood_deg(target_hood_angle);
    at_target_yaw_angle = reached_target_turret_yaw_deg(target_yaw_angle);
    at_target_limelight_angle = true;//reached_limelight_position(limelight_tx);
    inside_distance_window = distance < 5.5 && distance > 2.5;
    spin_up_clearance = allowed_to_shoot && inside_distance_window && limelightHasTarget && at_target_limelight_angle && at_target_yaw_angle && at_target_hood_angle;
    shoot_clearance = allowed_to_shoot && spin_up_clearance && at_target_shooter_rpm;

    about_to_shoot = turret_state == TurretStates::SPIN_UP_SHOOTER || turret_state == TurretStates::SHOOT;



    switch (turret_state)
    {
    case TurretStates::MANUAL:
    {
        set_turret_angle(target_manual_yaw_angle);
        Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, target_manual_shooter_rpm, 0);
        Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, target_manual_hood_angle / 360.0, 0);
        break;

        // use operator controls to aim turret
    }
    case TurretStates::TRACKING:
    {
        turn_limelight_off();
        
        set_turret_angle(angle);
        set_hood_distance(distance);
        turn_shooter_off();

        break;

        // aim turret

        // adjust hood
    }
    case TurretStates::AIM:
    {
        turn_limelight_on();

        set_turret_angle(angle);
        set_hood_distance(distance);
        turn_shooter_off();

        break;

        // turn on limelight

        // use limelight to aim

        // set hood based on limelight

        // spin up wheel
    }
    case TurretStates::TARGET_LOCKED:
    {
        turn_limelight_on();

        set_turret_angle(angle);
        set_hood_distance(distance);
        turn_shooter_off();

        break;
    }
    case TurretStates::SPIN_UP_SHOOTER:
    {

        turn_limelight_on();

        set_turret_angle(angle);
        set_hood_distance(distance);
        set_shooter_vel(distance);

        break;

        // sets shooter vel
    }
    case TurretStates::SHOOT:
    {

        turn_limelight_on();

        set_turret_angle(angle);
        set_hood_distance(distance);
        set_shooter_vel(distance);
        controlMsg.command_shoot = true;
        break;

        // shoot the ball
    }
    case TurretStates::PREPARE_CLIMB:
    {
        Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
        Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
        break;
    }
    }

    switch (turret_state)
    {
    case TurretStates::MANUAL:
    {
        if (!manual_control_enabled)
        {
            next_turret_state = TurretStates::TRACKING;
        }
        break;

        // use operator controls to aim turret
    }
    case TurretStates::TRACKING:
    {
        if (readyToShoot)
        {
            next_turret_state = TurretStates::AIM;
        }
        break;

        // aim turret

        // adjust hood
    }
    case TurretStates::AIM:
    {
        if (limelightHasTarget)
        {
            next_turret_state = TurretStates::TARGET_LOCKED;
        }
        break;

        // turn on limelight

        // use limelight to aim

        // set hood based on limelight

        // spin up wheel
    }
    case TurretStates::TARGET_LOCKED:
    {
        if (spin_up_clearance)
        {
            next_turret_state = TurretStates::SPIN_UP_SHOOTER;
        }

        break;
    }
    case TurretStates::SPIN_UP_SHOOTER:
    {
        if (shoot_clearance)
        {
            next_turret_state = TurretStates::SHOOT;
        }
        if(!allowed_to_shoot)
        {
            next_turret_state = TurretStates::TRACKING;
        }

        break;

        // sets shooter vel
    }
    case TurretStates::SHOOT:
    {
        if (time_in_state > ros::Duration(.75))
        {
            next_turret_state = TurretStates::TRACKING;
        }

        break;

        // shoot the ball
        
    }
    case TurretStates::PREPARE_CLIMB:
    {
        next_turret_state = TurretStates::PREPARE_CLIMB;    //Stay here forever
        break;
    }
    }

    intakeControlPublisher.publish(controlMsg);
}

void config_motors()
{
    Turret_Shooter_Master = new Motor(TURRET_SHOOTER_MASTER_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Shooter_Slave_Motor = new Motor(TURRET_SHOOTER_SLAVE_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Yaw_Motor = new Motor(TURRET_YAW_CAN_ID, Motor::Motor_Type::TALON_FX);
    Turret_Hood_Motor = new Motor(TURRET_HOOD_CAN_ID, Motor::Motor_Type::TALON_FX);

    Turret_Yaw_Motor->config().set_kP(2.6);
    Turret_Yaw_Motor->config().set_kI(0.0);
    Turret_Yaw_Motor->config().set_kD(320.0);
    Turret_Yaw_Motor->config().set_kF(0.074611);
    Turret_Yaw_Motor->config().set_motion_cruise_velocity(17000);
    Turret_Yaw_Motor->config().set_motion_acceleration(26000);
    Turret_Yaw_Motor->config().set_motion_s_curve_strength(5);
    Turret_Yaw_Motor->config().set_forward_soft_limit(0.9);
    Turret_Yaw_Motor->config().set_forward_soft_limit_enable(true);
    Turret_Yaw_Motor->config().set_reverse_soft_limit(-0.9);
    Turret_Yaw_Motor->config().set_reverse_soft_limit_enable(true);
    Turret_Yaw_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Yaw_Motor->config().set_supply_current_limit(true, 25, 0, 0);
    Turret_Yaw_Motor->config().apply();

    Turret_Hood_Motor->config().set_kP(0.8);
    Turret_Hood_Motor->config().set_kI(0.0);
    Turret_Hood_Motor->config().set_kD(120);
    Turret_Hood_Motor->config().set_kF(0.047651);
    Turret_Hood_Motor->config().set_motion_cruise_velocity(10000);
    Turret_Hood_Motor->config().set_motion_acceleration(32000);
    Turret_Hood_Motor->config().set_motion_s_curve_strength(5);
    Turret_Hood_Motor->config().set_forward_soft_limit(21.0/360.0);
    Turret_Hood_Motor->config().set_forward_soft_limit_enable(true);
    Turret_Hood_Motor->config().set_reverse_soft_limit(0);
    Turret_Hood_Motor->config().set_reverse_soft_limit_enable(true);
    Turret_Hood_Motor->config().set_supply_current_limit(true, 5, 0, 0);
    Turret_Hood_Motor->config().set_inverted(true);
    Turret_Hood_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Hood_Motor->config().apply();

    Turret_Shooter_Slave_Motor->config().set_follower(true, TURRET_SHOOTER_MASTER_CAN_ID);
    Turret_Shooter_Slave_Motor->config().set_inverted(true);
    Turret_Shooter_Slave_Motor->config().set_supply_current_limit(true, 40, 20, 1);
    Turret_Shooter_Slave_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Shooter_Slave_Motor->config().set_peak_output_reverse(-0.5);
    Turret_Shooter_Slave_Motor->config().apply();

    Turret_Shooter_Master->config().set_kP(0.023);
    Turret_Shooter_Master->config().set_kI(0.0);
    Turret_Shooter_Master->config().set_kD(1);
    Turret_Shooter_Master->config().set_kF(0.0466);
    Turret_Shooter_Master->config().set_closed_loop_ramp(0.25);
    Turret_Shooter_Master->config().set_peak_output_reverse(-0.5);
    Turret_Shooter_Master->config().set_supply_current_limit(true, 40, 20, 1);
    Turret_Shooter_Master->config().apply();
}

void motor_status_callback(const rio_control_node::Motor_Status &msg)
{
    (void)msg;
    // double motor_rotations = 0;
    // bool found_motor = false;
    static std::map<uint8_t, rio_control_node::Motor_Info> motorInfoMap;
    motorInfoMap.clear();

    for (std::vector<rio_control_node::Motor_Info>::const_iterator i = msg.motors.begin();
         i != msg.motors.end();
         i++)
    {
        motorInfoMap[(*i).id] = (*i);
    }

    if (motorInfoMap.count(TURRET_SHOOTER_MASTER_CAN_ID))
    {
        actualShooterRPM = motorInfoMap[TURRET_SHOOTER_MASTER_CAN_ID].sensor_velocity;
    }

    if (motorInfoMap.count(TURRET_HOOD_CAN_ID))
    {
        actualHoodDeg = motorInfoMap[TURRET_HOOD_CAN_ID].sensor_position * 360;
    }

    if (motorInfoMap.count(TURRET_YAW_CAN_ID))
    {
        actualTurretYawDeg = motorInfoMap[TURRET_YAW_CAN_ID].sensor_position * 360;
        
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "turret_link";

        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 14 * INCHES_TO_METERS;

        tf2::Quaternion q;
        q.setRPY(0, 0, motorInfoMap[TURRET_YAW_CAN_ID].sensor_position * 2.0 * M_PI);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tfBroadcaster->sendTransform(transformStamped);
    }
}

void publish_pose(ros::Publisher publisher, float angle)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0,angle * M_PI / 180.0);

    pose.pose.orientation.w = q.getW();
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();

    publisher.publish(pose);

}

void publish_diagnostic_data()
{
    static ros::Publisher diagnostic_publisher = node->advertise<turret_node::turret_diagnostics>("/TurretNodeDiagnostics", 1);
    turret_node::turret_diagnostics diagnostics;
    diagnostics.turret_state = turret_state_to_string(turret_state);
    diagnostics.next_turret_state = turret_state_to_string(next_turret_state);
    diagnostics.actual_shooter_rpm = actualShooterRPM;
    diagnostics.actual_hood_angle = actualHoodDeg;
    diagnostics.actual_yaw_angle = actualTurretYawDeg;
    diagnostics.limelightHasTarget = limelightHasTarget;
    diagnostics.readyToShoot = readyToShoot;
    diagnostics.target_hood_angle = target_hood_angle;
    diagnostics.target_shooter_rpm = target_shooter_rpm;
    diagnostics.target_yaw_angle = target_yaw_angle;
    diagnostics.target_shooter_offset = target_vel_offset;
    diagnostics.target_hood_offset = target_hood_offset;
    diagnostics.target_yaw_offset = target_yaw_offset;
    diagnostics.at_target_shooter_rpm = at_target_shooter_rpm;
    diagnostics.at_target_hood_angle = at_target_hood_angle;
    diagnostics.at_target_limelight_angle = at_target_limelight_angle;
    diagnostics.at_target_yaw_angle = at_target_yaw_angle;
    diagnostics.at_shooter_rpm_time = at_shooter_rpm_time;
    diagnostics.spin_up_clearance = spin_up_clearance;
    diagnostics.shoot_clearance = shoot_clearance;
    diagnostics.allowed_to_shoot = allowed_to_shoot;
    diagnostics.about_to_shoot = about_to_shoot;
    diagnostics.limelight_tx = limelight_tx;

    if(limelightHasTarget)
    {
        diagnostics.robot_distance = get_distance_to_hub_limelight();
    }
    else
    {
        diagnostics.robot_distance = get_distance_to_hub();
    }
    diagnostic_publisher.publish(diagnostics);

    float odometry_angle = get_angle_to_hub();
    float limelight_angle = get_angle_to_hub_limelight();

    static ros::Publisher odometry_angle_publisher = node->advertise<geometry_msgs::PoseStamped>("/odometry_hub_angle", 1);
    static ros::Publisher limelight_angle_publisher = node->advertise<geometry_msgs::PoseStamped>("/limelight_hub_angle", 1);

    publish_pose (odometry_angle_publisher, odometry_angle);
    publish_pose (limelight_angle_publisher, limelight_angle);
}

void publish_turret_status()
{
    static ros::Publisher status_publisher = node->advertise<turret_node::turret_status>("/TurretStatus", 1);
    turret_node::turret_status status;
    status.about_to_shoot = about_to_shoot;

    status_publisher.publish(status);
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
    while (ros::ok())
    {
        ros::spinOnce();

        step_state_machine();
        publish_diagnostic_data();
        publish_turret_status();

        rate.sleep();
    }

    return 0;
}
