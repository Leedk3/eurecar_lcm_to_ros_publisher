// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include "eurecar_lcm_to_ros_publisher/lcmtoros_can_t.hpp"
#include "eurecar_lcm_to_ros_publisher/lcmtoros_pos_t.hpp"

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    ros::init(argc, argv, "LCM_to_ROS_republish");
    ros::NodeHandle nh;   

    //LCM handler 
    LCMToROS_CAN_T can_t_handlerObject(nh);
    LCMToROS_POS_T pos_t_handlerObject(nh); // Odometry publisher for EKF pose 
    
    //lcm subscriber
    lcm.subscribe("CAN_T", &LCMToROS_CAN_T::lcmCallback, &can_t_handlerObject);
    lcm.subscribe("POS_T", &LCMToROS_POS_T::lcmCallback, &pos_t_handlerObject);

    int lcm_timeout = 100; //ms
    while( ( lcm.handleTimeout(lcm_timeout) >= 0 ) && (ros::ok()) ) {
        
    }
            
    ros::spinOnce();

    return 0;
}
