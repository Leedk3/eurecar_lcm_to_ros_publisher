#include "eurecar_lcm_to_ros_publisher/lcmtoros_pos_t.hpp"
#include <ros/ros.h>

LCMToROS_POS_T::LCMToROS_POS_T(ros::NodeHandle& n) 
{
    nh = n;
    // Publishers
    ekf_odom_pub = nh.advertise<nav_msgs::Odometry>("/Odometry/ekf_pose", 10);
    ROS_DEBUG("pos_t publisher created");
};

LCMToROS_POS_T::~LCMToROS_POS_T() 
{    
    ROS_INFO("pos_t LCMToROS_POS_T destructor.");
}

void LCMToROS_POS_T::lcmCallback(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel_name,
        const eurecar::pos_t* msg)
{
    ROS_DEBUG("Received message on channel ");
    ekf_odom_msg.header.stamp = ros::Time::now();
    ekf_odom_msg.header.frame_id = "odom";
    ekf_odom_msg.child_frame_id = "base_link";
    ekf_odom_msg.pose.pose.position.x = msg -> x;
    ekf_odom_msg.pose.pose.position.y = msg -> y;
    ekf_odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion quat_ekf;
    geometry_msgs::Quaternion quat_ekf_msg;
    double angle_z_rad = msg -> h; // M_PI /2 - msg -> h; //check pos_t is rad or deg
    if (std::isnan(angle_z_rad)){
        angle_z_rad = 0;
    }
    quat_ekf.setRPY(0,0,angle_z_rad);
    quat_ekf.normalize();
    quat_ekf_msg = tf2::toMsg(quat_ekf);

    ekf_odom_msg.pose.pose.orientation = quat_ekf_msg;

    ekf_odom_msg.twist.twist.linear.x = msg -> v;
    ekf_odom_msg.twist.twist.angular.z = msg -> yaw_rate;

    ekf_odom_pub.publish(ekf_odom_msg);

    // TF publisher
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(ekf_odom_msg.pose.pose.position.x, ekf_odom_msg.pose.pose.position.y, ekf_odom_msg.pose.pose.position.z) ); 
    transform.setRotation(tf::Quaternion(0, 0, angle_z_rad) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

};