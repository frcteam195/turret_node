#include "turret_node.hpp"
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
#include "turret_node/Turret_Diagnostics.h"
#include "turret_node/Turret_Status.h"
#include "climber_node/Turret_Ready.h"
#include <thread>
#include <string>
#include <mutex>

#include <action_helper/action_helper.hpp>

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Motor_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "ck_utilities/NTHelper.hpp"
#include "ck_utilities/MovingAverage.hpp"
#include "ck_utilities/RateControlledPublisher.hpp"

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
ck::ros::RateControlledPublisher<limelight_vision_node::Limelight_Control>* m_limelight_control_pub;
ck::MovingAverage mShooterRPMAverage(5);

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

static constexpr double SHOOTER_RPM_DELTA = 150;
static constexpr float HOOD_DEG_DELTA = 2;
static constexpr float TURRET_YAW_DEG_DELTA = 2;
static constexpr float SHOOTER_RPM_FILTER_TIME = 0.15;


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
static bool ready_to_climb = false;
static float limelight_tx = 0;
static float at_shooter_rpm_time = 0;
static float robot_distance = 0;

static float shuffleboard_offset = 0;
static float shuffleboard_angle_offset = 0;

static bool hooks_deployed = false;

static double robot_rotation_rate_rad_per_sec = 0;
static constexpr double MAX_ROTATION_RATE_RAD_PER_SEC = 8.9;
static double turret_arbFF = 0;

static constexpr double TURRET_GEAR_RATIO = 26.875;
static constexpr double TURRET_CRUISE_VEL_TICKS_PER_100MS = 21000.0;
static constexpr double TURRET_MAX_YAW_RATE_RAD_PER_SEC = TURRET_CRUISE_VEL_TICKS_PER_100MS / 2048.0 * 600.0 / TURRET_GEAR_RATIO / 60.0 * 2.0 * ck::math::PI;

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
        theta = ck::math::rad2deg(atan2(y, x));
        return theta;
    }
    catch (...)
    {
        static ros::Time prevPubTime(0);
        if (ros::Time::now() - prevPubTime > ros::Duration(1))
        {
            ROS_WARN("Hub transform failed");
            prevPubTime = ros::Time::now();
        }
    }
    return 0;
}

float lookup_corrected_distance(float input)
{
    float input_inches = input / INCHES_TO_METERS;
    static bool first = true;
    static InterpolatingMap<float, float> distance_correction_map;
    if (first)
    {
        distance_correction_map.insert(0.0, 0.0);
        distance_correction_map.insert(108.0, 108.228);
        distance_correction_map.insert(132.0, 133.248);
        distance_correction_map.insert(156.0, 157.97);
        distance_correction_map.insert(180.0, 185.354);
        distance_correction_map.insert(204.0, 208.189);
        distance_correction_map.insert(228.0, 238.779);
        // distance_correction_map.insert(252, 270.334); this was an outlier
        distance_correction_map.insert(276.0, 288.878);
        distance_correction_map.insert(380.0, 397.730);
        distance_correction_map.insert(450.0, 470.996);
        first = false;
    }
    // ROS_INFO("Inches Input: %f, output: %f", input_inches, distance_correction_map.lookup(input_inches) * INCHES_TO_METERS);
    return distance_correction_map.lookup(input_inches) * INCHES_TO_METERS;
    
}

float get_distance_to_hub()
{
    tf2::Stamped<tf2::Transform> robot_base_to_hub;

    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "hub_link", ros::Time(0)), robot_base_to_hub);
        return lookup_corrected_distance(robot_base_to_hub.getOrigin().length());
    }

    catch (...)
    {
        static ros::Time prevPubTime(0);
        if (ros::Time::now() - prevPubTime > ros::Duration(1))
        {
            ROS_WARN("Hub transform failed");
            prevPubTime = ros::Time::now();
        }
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
        theta = ck::math::rad2deg(atan2(y, x));
        return theta;
    }

    catch (...)
    {
        static ros::Time prevPubTime(0);
        if (ros::Time::now() - prevPubTime > ros::Duration(1))
        {
            ROS_WARN("Hub transform failed");
            prevPubTime = ros::Time::now();
        }
    }
    return 0;
}

float get_distance_to_hub_limelight()
{
    tf2::Stamped<tf2::Transform> limelight_link_hub;

    try
    {
        tf2::convert(tfBuffer.lookupTransform("base_link", "limelight_link_hub", ros::Time(0)), limelight_link_hub);
        return lookup_corrected_distance(sqrt(pow(limelight_link_hub.getOrigin().getX(), 2) + pow(limelight_link_hub.getOrigin().getY(), 2)));
    }

    catch (...)
    {
        static ros::Time prevPubTime(0);
        if (ros::Time::now() - prevPubTime > ros::Duration(1))
        {
            ROS_WARN("Hub transform failed");
            prevPubTime = ros::Time::now();
        }
    }
    return 0;
}

void turn_limelight_on()
{
    limelight_vision_node::Limelight limelight;
    limelight.name = "limelight";
    limelight.pipeline = 1;

    limelight_vision_node::Limelight_Control limelight_control;
    limelight_control.limelights.push_back(limelight);

    if (m_limelight_control_pub)
    {
        m_limelight_control_pub->publish_at_rate(limelight_control, 20);
    }
    else
    {
        ROS_ERROR("Failed to send limelight control msg");
    }
}

void turn_limelight_off()
{
    limelight_vision_node::Limelight limelight;
    limelight.name = "limelight";
    limelight.pipeline = 1;

    limelight_vision_node::Limelight_Control limelight_control;
    limelight_control.limelights.push_back(limelight);

    if(m_limelight_control_pub)
    {
        m_limelight_control_pub->publish_at_rate(limelight_control, 20);
    }
    else
    {
        ROS_ERROR("Failed to send limelight control msg");
    }
}

void intake_status_callback(const intake_node::Intake_Status &msg)
{
    readyToShoot = msg.readyToShoot;
}

void odometry_callback(const nav_msgs::Odometry &msg)
{
    //Max angular rate is 8.9rad/s
    robot_rotation_rate_rad_per_sec = msg.twist.twist.angular.z; //rad/s
}

void hmi_signal_callback(const hmi_agent_node::HMI_Signals &msg)
{
    target_manual_hood_angle = msg.turret_hood_degrees;
    target_manual_shooter_rpm = msg.turret_speed_rpm;
    target_manual_yaw_angle = msg.turret_aim_degrees;
    manual_control_enabled = msg.turret_manual;
    allowed_to_shoot = msg.allow_shoot;
    static bool last_increase_offset = false;
    if(msg.increase_offset && !last_increase_offset)
    {
        shuffleboard_offset += 10.0;
    }
    last_increase_offset = msg.increase_offset;
    static bool last_decrease_offset = false;
    if(msg.decrease_offset && !last_decrease_offset)
    {
        shuffleboard_offset -= 10.0;
    }
    last_decrease_offset = msg.decrease_offset;
    if (!hooks_deployed)
    {
        hooks_deployed = msg.deploy_hooks;
    }

    static bool last_angle_increase_offset = false;
    if(msg.angle_increase_offset && !last_angle_increase_offset)
    {
        shuffleboard_angle_offset += 1.0;
    }
    last_angle_increase_offset = msg.angle_increase_offset;

    static bool last_angle_decrease_offset = false;
    if(msg.angle_decrease_offset && !last_angle_decrease_offset)
    {
        shuffleboard_angle_offset -= 1.0;
    }
    last_angle_decrease_offset = msg.angle_decrease_offset;

    // still needs to be updated
}

void limelight_status_callback(const limelight_vision_node::Limelight_Status &msg)
{
    if (msg.limelights.size() > 0)
    {
        limelightHasTarget = msg.limelights[0].target_valid;
        limelight_tx = msg.limelights[0].target_dx_deg;
    }
}

static float target_vel_offset = 0;
static float target_hood_offset = 0;
static float target_yaw_offset = 0;
bool reached_target_vel(float targetVel)
{

    target_vel_offset = fabs(targetVel - actualShooterRPM);
    mShooterRPMAverage.addSample(targetVel - actualShooterRPM);
    return ck::math::inRange(mShooterRPMAverage.getAverage(), SHOOTER_RPM_DELTA);
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
        hood_lookup_table.insert(1.161,0);
        hood_lookup_table.insert(1.473,0.005);
        hood_lookup_table.insert(1.768,0.0125);
        hood_lookup_table.insert(2.35,0.0215);
        hood_lookup_table.insert(2.728,0.03);
        hood_lookup_table.insert(3.145,0.0355);
        hood_lookup_table.insert(3.38,0.0385);
        hood_lookup_table.insert(3.72,0.0415);
        hood_lookup_table.insert(3.96,0.044);
        hood_lookup_table.insert(4.37,0.0455);
        hood_lookup_table.insert(4.65,0.0475);
        hood_lookup_table.insert(4.99,0.05);
        hood_lookup_table.insert(5.52,0.0515);
        hood_lookup_table.insert(5.79,0.0515);
        hood_lookup_table.insert(6.208,0.0535);
        hood_lookup_table.insert(7.13,0.0575);
        hood_lookup_table.insert(7.66,0.0615);
        hood_lookup_table.insert(7.93,0.0615);
        hood_lookup_table.insert(8.37,0.06375);
        hood_lookup_table.insert(8.99,0.06375);
        hood_lookup_table.insert(9.19,0.06375);
        hood_lookup_table.insert(9.4,0.06375);
        hood_lookup_table.insert(10.06375,0.06375);
        hood_lookup_table.insert(10.26,0.06375);
        hood_lookup_table.insert(10.8,0.06375);
        first_time = false;
    }
    target_hood_angle = (hood_lookup_table.lookup(distance) * 360.0) + shuffleboard_angle_offset;
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, target_hood_angle / 360.0, 0);
}

void set_shooter_vel(float distance)
{
    static InterpolatingMap<float, float> shooter_rpm_lookup_table;
    static bool first_time = true;
    if (first_time)
    {
        shooter_rpm_lookup_table.insert(1.161,1125);
        shooter_rpm_lookup_table.insert(1.473,1125);
        shooter_rpm_lookup_table.insert(1.768,1200); // MGT - tweak this up a bit - used to be 1175
        shooter_rpm_lookup_table.insert(2.35,1235); // MGT - tweak this up a bit - used to be 1200
        shooter_rpm_lookup_table.insert(2.728,1285); // MGT - tweak this up a bit - used to be 1250
        shooter_rpm_lookup_table.insert(3.145,1335); // MGT - tweak this up a bit - used to be 1300
        shooter_rpm_lookup_table.insert(3.38,1395); // MGT - tweak this up a bit - used to be 1365
        shooter_rpm_lookup_table.insert(3.72,1425); // MGT - tweak this up a bit - used to be 1400
        shooter_rpm_lookup_table.insert(3.96,1475); // MGT - tweak this up a bit - used to be 1460
        shooter_rpm_lookup_table.insert(4.37,1535); // MGT - tweak this up a bit - used to be 1525
        shooter_rpm_lookup_table.insert(4.65,1600);
        shooter_rpm_lookup_table.insert(4.99,1630); // MGT - tweak this down a bit - used to be 1725
        shooter_rpm_lookup_table.insert(5.18,1640); // RAH - add point to smooth
        shooter_rpm_lookup_table.insert(5.52,1650); // MGT - tweak this down a bit - used to be 1800
        shooter_rpm_lookup_table.insert(5.79,1660); // RAH - tweak this down a bit - used to be 1850
        shooter_rpm_lookup_table.insert(6.208,1680); // RAH - tweak this down a bit - used to be 1900
        shooter_rpm_lookup_table.insert(6.48,1700); // RAH - add point to smooth
        shooter_rpm_lookup_table.insert(6.7,1710); // RAH - add point to smooth
        shooter_rpm_lookup_table.insert(7.13,1890); // RAH - tweak this down a bit - used to be 1950
        shooter_rpm_lookup_table.insert(7.66,2025); // MGT - tweak this up a bit - used to be 2050
        shooter_rpm_lookup_table.insert(7.93,2110); // MGT - tweak this up a bit - used to be 2150
        shooter_rpm_lookup_table.insert(8.37,2200);
        shooter_rpm_lookup_table.insert(8.99,2300);
        shooter_rpm_lookup_table.insert(9.19,2375);
        shooter_rpm_lookup_table.insert(9.4,2400);
        shooter_rpm_lookup_table.insert(10.065,2500);
        shooter_rpm_lookup_table.insert(10.26,2600);
        shooter_rpm_lookup_table.insert(10.8,2625);

        first_time = false;
    }
    target_shooter_rpm = shooter_rpm_lookup_table.lookup(distance) + shuffleboard_offset;
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, target_shooter_rpm, 0);
}

void turn_shooter_off()
{
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, 0, 0);
}
enum class BiasDirection
{
    LEFT, 
    RIGHT,
};

float calculate_turret_angle(float angleDeg, float prevAngle)
{
    BiasDirection bias = BiasDirection::LEFT;
    
    if (prevAngle > -90)
    {
        bias = BiasDirection::LEFT;
    }
    else
    {
        bias = BiasDirection::RIGHT;
    }
    
    float offset = bias == BiasDirection::RIGHT ? -15 : 15;
    target_yaw_angle = angleDeg;

    while(target_yaw_angle > 90 + offset)
    {
        target_yaw_angle -= 360.0;
    }
    while(target_yaw_angle < -270.0 + offset)
    {
        target_yaw_angle += 360.0;
    }

   
    return target_yaw_angle;
}

void set_turret_angle(float angleDeg)
{
    static float prev_target_angle = 0;
    float target_angle = calculate_turret_angle(angleDeg, prev_target_angle);
    prev_target_angle = target_angle;

    if (std::abs(robot_rotation_rate_rad_per_sec) > 0.2)
    {
        double rawPercentRotation = -(robot_rotation_rate_rad_per_sec / TURRET_MAX_YAW_RATE_RAD_PER_SEC) * 6.0;
        rawPercentRotation = ck::math::signum(rawPercentRotation) * std::min(std::abs(rawPercentRotation), 1.0);

        int turret_direction_signum = ck::math::signum(target_angle - actualTurretYawDeg);

        turret_arbFF = ck::math::signum(rawPercentRotation) != turret_direction_signum ? 0 : rawPercentRotation;
    }
    else
    {
        turret_arbFF = 0;
    }

    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, target_angle / 360.0, turret_arbFF);
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
    inside_distance_window = true;//distance < 5.5 && distance > 2.5;
    spin_up_clearance = allowed_to_shoot && inside_distance_window /*&& limelightHasTarget*/ && at_target_limelight_angle && at_target_yaw_angle && at_target_hood_angle;
    shoot_clearance = allowed_to_shoot && spin_up_clearance && at_target_shooter_rpm;

    about_to_shoot = turret_state == TurretStates::SPIN_UP_SHOOTER || turret_state == TurretStates::SHOOT;

    static float locked_in_shoot_distance = 0;
    static float locked_in_angle = 0;

    switch (turret_state)
    {
    case TurretStates::MANUAL:
    {
        // set_turret_angle(target_manual_yaw_angle);
        Turret_Yaw_Motor->set(Motor::Control_Mode::PERCENT_OUTPUT, target_manual_yaw_angle, 0);
        Turret_Yaw_Motor->config().set_forward_soft_limit_enable(false);
        Turret_Yaw_Motor->config().set_reverse_soft_limit_enable(false);
        Turret_Yaw_Motor->config().apply();
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

        locked_in_shoot_distance = distance;
        locked_in_angle = angle;

        break;

        // sets shooter vel
    }
    case TurretStates::SHOOT:
    {

        turn_limelight_on();

        distance = locked_in_shoot_distance;
        angle = locked_in_angle;

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
        if (reached_target_turret_yaw_deg(0))
        {
            ready_to_climb = true;
        }
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
        // if (limelightHasTarget)
        // {
            next_turret_state = TurretStates::TARGET_LOCKED;
        // }
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
        if (time_in_state > ros::Duration(0.7))
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

    robot_distance = distance;
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
    Turret_Yaw_Motor->config().set_motion_cruise_velocity(TURRET_CRUISE_VEL_TICKS_PER_100MS);
    Turret_Yaw_Motor->config().set_motion_acceleration(38000);
    Turret_Yaw_Motor->config().set_motion_s_curve_strength(5);
    Turret_Yaw_Motor->config().set_forward_soft_limit(0.55);
    Turret_Yaw_Motor->config().set_forward_soft_limit_enable(true);
    Turret_Yaw_Motor->config().set_reverse_soft_limit(-1.0);
    Turret_Yaw_Motor->config().set_reverse_soft_limit_enable(true);
    Turret_Yaw_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Yaw_Motor->config().set_supply_current_limit(true, 40, 0, 0);
    Turret_Yaw_Motor->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
    Turret_Yaw_Motor->config().apply();
    Turret_Yaw_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);

    Turret_Hood_Motor->config().set_kP(0.8);
    Turret_Hood_Motor->config().set_kI(0.0);
    Turret_Hood_Motor->config().set_kD(120);
    Turret_Hood_Motor->config().set_kF(0.047651);
    Turret_Hood_Motor->config().set_motion_cruise_velocity(10000);
    Turret_Hood_Motor->config().set_motion_acceleration(32000);
    // Turret_Hood_Motor->config().set_motion_s_curve_strength(5);
    Turret_Hood_Motor->config().set_forward_soft_limit(23/360.0);
    Turret_Hood_Motor->config().set_forward_soft_limit_enable(true);
    Turret_Hood_Motor->config().set_reverse_soft_limit(0);
    Turret_Hood_Motor->config().set_reverse_soft_limit_enable(true);
    Turret_Hood_Motor->config().set_supply_current_limit(true, 5, 0, 0);
    Turret_Hood_Motor->config().set_inverted(true);
    Turret_Hood_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Hood_Motor->config().apply();
    Turret_Hood_Motor->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);

    Turret_Shooter_Slave_Motor->config().set_follower(true, TURRET_SHOOTER_MASTER_CAN_ID);
    Turret_Shooter_Slave_Motor->config().set_inverted(true);
    Turret_Shooter_Slave_Motor->config().set_supply_current_limit(true, 40, 20, 1);
    Turret_Shooter_Slave_Motor->config().set_closed_loop_ramp(0.25);
    Turret_Shooter_Slave_Motor->config().set_peak_output_reverse(-0.5);
    Turret_Shooter_Slave_Motor->config().apply();

    Turret_Shooter_Master->config().set_kP(0.014);
    Turret_Shooter_Master->config().set_kI(0.0);
    Turret_Shooter_Master->config().set_kD(2.4);
    Turret_Shooter_Master->config().set_kF(0.0465);
    Turret_Shooter_Master->config().set_closed_loop_ramp(0.25);
    Turret_Shooter_Master->config().set_peak_output_reverse(-0.5);
    Turret_Shooter_Master->config().set_supply_current_limit(true, 40, 20, 1);
    Turret_Shooter_Master->config().apply();
    Turret_Shooter_Master->set(Motor::Control_Mode::VELOCITY, 0, 0);

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
        transformStamped.transform.translation.z = (31 + (7/8)) * INCHES_TO_METERS;

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
    static ros::Publisher diagnostic_publisher = node->advertise<turret_node::Turret_Diagnostics>("/TurretNodeDiagnostics", 1);
    turret_node::Turret_Diagnostics diagnostics;
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
    diagnostics.turret_arbFF = turret_arbFF;
    diagnostics.shuffleboard_offset = shuffleboard_offset;
    diagnostics.shuffleboard_angle_offset = shuffleboard_angle_offset;
    diagnostics.robot_distance = robot_distance;
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
    static ros::Publisher status_publisher = node->advertise<turret_node::Turret_Status>("/TurretStatus", 1);
    static ros::Publisher turret_climb_ready_publisher = node->advertise<climber_node::Turret_Ready>("/TurretClimbReady", 1);
    turret_node::Turret_Status status;
    climber_node::Turret_Ready turretClimbStatus;
    status.about_to_shoot = about_to_shoot;
    turretClimbStatus.ready_to_climb = ready_to_climb;

    status_publisher.publish(status);
    turret_climb_ready_publisher.publish(turretClimbStatus);
}



void publish_shuffleboard_data()
{
    static std::string table_name = "dashboard_data";
    ck::nt::set(table_name, "spin_up_clearance", spin_up_clearance);
    ck::nt::set(table_name, "shooter_rpm", actualShooterRPM);
    ck::nt::set(table_name, "hood_angle", actualHoodDeg);
    ck::nt::set(table_name, "turret_angle", actualTurretYawDeg);
    ck::nt::set(table_name, "robot_distance", robot_distance);
    ros::Time last_valid;
    // ck::nt::get(shuffleboard_offset, last_valid, table_name, "shuffleboard_offset", (float) 0.0);
    ck::nt::set(table_name, "live_shuffleboard_offset", shuffleboard_offset);
    ck::nt::set(table_name, "live_shuffleboard_angle_offset", shuffleboard_angle_offset);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turret_node");
    ros::NodeHandle n;
    node = &n;

    m_limelight_control_pub = new ck::ros::RateControlledPublisher<limelight_vision_node::Limelight_Control>("/LimelightControl");

    config_motors();

    ros::Subscriber hmi_subscribe = node->subscribe("/HMISignals", 1, hmi_signal_callback);
    ros::Subscriber motor_status_subscribe = node->subscribe("/MotorStatus", 1, motor_status_callback);
    ros::Subscriber limelight_status_subscribe = node->subscribe("/LimelightStatus", 1, limelight_status_callback);
    ros::Subscriber intake_status_subscribe = node->subscribe("/IntakeStatus", 1, intake_status_callback);
    ros::Subscriber odometry_subscribe = node->subscribe("/odometry/filtered", 1, odometry_callback);

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
        publish_shuffleboard_data();
    
        rate.sleep();
    }

    return 0;
}
