// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
// Include ROS message_type which will be published
#include "eurecar_lcm_to_ros_publisher/eurecar_can_t.h"

// include LCM message --> user definition
#include "eurecar_lcm_to_ros_publisher/lcm/eurecar/can_t.hpp"


class LCMToROS_CAN_T
{
    public:
        LCMToROS_CAN_T(ros::NodeHandle& n);
        ~LCMToROS_CAN_T();
        void lcmCallback(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel_name,
                const eurecar::can_t* msg);        
    private:        
        ros::NodeHandle nh;
        ros::Publisher ros_can_t_pub;
        eurecar_lcm_to_ros_publisher::eurecar_can_t can_t_msg;
};