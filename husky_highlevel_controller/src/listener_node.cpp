// ROS C++ header
#include <ros/ros.h>
#include "husky_highlevel_controller/listener.hpp"

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "listener");
    ros::NodeHandle nodeHandle("~");

    listener_controller::ListenerController ListenerController(nodeHandle);

    ros::spin();
    return 0;
}