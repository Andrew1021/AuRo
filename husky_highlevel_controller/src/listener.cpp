#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String & msg)
{
    ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nodeHandle;
    
    // recieve of topics with method from node handle
    // callback with data is started when message arrvies
    // stay connected with subscriber object until no message should be recieved
    ros::Subscriber subscriber = nodeHandle.subscribe("chatter", 10, chatterCallback);

    // continues call of callback until node ist terminated
    ros::spin();

    return 0;

}