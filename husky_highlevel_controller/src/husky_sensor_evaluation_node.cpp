#include <ros/ros.h>
#include "husky_highlevel_controller/HuskySensorEvaluation.hpp"

int main(int argc, char ** argv)
{
    // must called before other ROS function 
    ros::init(argc, argv, "husky_sensor_evaluation_node");
    ros::NodeHandle nodeHandle("~");
    
    husky_highlevel_controller::HuskySensorEvaluation HuskySensorEvaluation(nodeHandle);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
    //ros::spin();

    return 0;
}