#include "eurecar_lcm_to_ros_publisher/lcmtoros_can_t.hpp"
#include <ros/ros.h>

LCMToROS_CAN_T::LCMToROS_CAN_T(ros::NodeHandle& n) 
{
    nh = n;
    // Publishers
    ros_can_t_pub = nh.advertise<eurecar_lcm_to_ros_publisher::eurecar_can_t>("/vehicle/can_t", 10);
    ROS_DEBUG("can_t publisher created");
};

LCMToROS_CAN_T::~LCMToROS_CAN_T() 
{    
    ROS_INFO("can_t LCMToROS_CAN_T destructor.");
}

void LCMToROS_CAN_T::lcmCallback(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel_name,
        const eurecar::can_t* msg)
{
    ROS_DEBUG("Received message on channel ");
    can_t_msg.header.stamp = ros::Time::now();
    can_t_msg.header.frame_id = "/base_footprint";
    can_t_msg.yaw_rate = msg -> yaw_rate;
    can_t_msg.mdps_torque = msg -> mdps_torque;
    can_t_msg.mdps_str_ang = msg -> mdps_str_ang;
    can_t_msg.VS_CAN = msg -> VS_CAN;
    can_t_msg.lat_accel = msg -> lat_accel;
    can_t_msg.mcp = msg -> mcp;
    can_t_msg.accel_pedal_value = msg -> accel_pedal_value;
    can_t_msg.tps = msg -> tps;
    can_t_msg.odometer = msg -> odometer;
    can_t_msg.WHL_SPD_RR = msg -> WHL_SPD_RR;
    can_t_msg.WHL_SPD_RL = msg -> WHL_SPD_RL;
    can_t_msg.WHL_SPD_FR = msg -> WHL_SPD_FR;
    can_t_msg.WHL_SPD_FL = msg -> WHL_SPD_FL;


    ros_can_t_pub.publish(can_t_msg);

};