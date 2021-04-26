// ROS C++ header
//#include <ros/ros.h>
// STD
#include <string>
#include "husky_highlevel_controller/hello_world.hpp"

namespace hello_world_controller
{

    HelloWorldController::HelloWorldController(ros::NodeHandle & nodeHandle) : nodeHandle_(nodeHandle)
    {
        if (!ros::ok()) {
            ROS_ERROR("Could not start node successfull.");
            ros::requestShutdown();
        }

        ROS_INFO("Successfully launched node.");

        printHello();
    }

    HelloWorldController::~HelloWorldController() {}

    void HelloWorldController::printHello(unsigned int count)
    {
        // header class for looping with specific frequency
        ros::Rate loopRate(10);

        count = 0;

        // check if node should run on (false if SIGINT (Ctrl + C) or ros::shutdown())
        while (ros::ok()) 
        {
            // shows messages
            ROS_INFO_STREAM("Hello World" << count);
            // processes incoming messages via callbacks
            ros::spinOnce();
            loopRate.sleep();
            count++;
        }
    }

} /* namespace */