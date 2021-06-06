#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyMotionController.hpp"

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "husky_motion_controller_node");
    ros::NodeHandle nodeHandle("~");
    
    husky_highlevel_controller::HuskyMotionController HuskyMotionController(nodeHandle);
    
    ros::spin();

    return 0;
}