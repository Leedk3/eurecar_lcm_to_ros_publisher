// inlcude C library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <cmath>

// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
// Include ROS message_type which will be published
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

// include LCM message --> user definition
#include "eurecar_lcm_to_ros_publisher/lcm/eurecar/pos_t.hpp"






class LCMToROS_POS_T
{
    public:
        LCMToROS_POS_T(ros::NodeHandle& n);
        ~LCMToROS_POS_T();
        void lcmCallback(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel_name,
                const eurecar::pos_t* msg);        
    private:        
        ros::NodeHandle nh;
        ros::Publisher ekf_odom_pub;
        nav_msgs::Odometry ekf_odom_msg;
};