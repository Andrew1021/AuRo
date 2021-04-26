// ROS C++ header
#include <ros/ros.h>
#include "husky_highlevel_controller/hello_world.hpp"

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "hello_world");
    ros::NodeHandle nodeHandle("~");

    hello_world_controller::HelloWorldController HelloWorldController(nodeHandle);

    ros::spin();
    return 0;
}