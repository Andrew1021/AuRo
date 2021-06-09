#include <ros/ros.h>
#include <husky_highlevel_controller/HuskyActionClient.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_action_client_node");
    ros::NodeHandle nh("~");

    husky_highlevel_controller::HuskyActionClient HuskyActionClient(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}