// ROS C++ header
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "hello_world");
    ros::NodeHandle nodeHandle;
    // helader class for looping with specific frequency
    ros::Rate loopRate(10);

    unsigned int count = 0;

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

    return 0;

}