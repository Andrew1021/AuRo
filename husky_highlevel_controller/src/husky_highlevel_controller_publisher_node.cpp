#include <ros/ros.h>
#include "husky_highlevel_controller/husky_highlevel_controller_publisher.hpp"

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "husky_highlevel_controller_publisher");
    ros::NodeHandle nodeHandle("~");
    
    husky_highlevel_controller_publisher::HuskyHighLevelControllerPublisher HuskyHighLevelControllerPublisher(nodeHandle);
    
    ros::spin();

    return 0;
}