#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nodeHandle;
    // create publisher with help of nodeHandle and define topic "chatter"
    ros::Publisher chatterPublisher = nodeHandle.advertise<std_msgs::String>("chatter", 1);

    ros::Rate loopRate(10);
    unsigned int count = 0;

    while(ros::ok())
    {
        std_msgs::String message;
        // define message content
        message.data = "Hello World " + std::to_string(count) + "!";
        ROS_INFO_STREAM(message.data);
        // publish message to topic "chatter"
        chatterPublisher.publish(message);

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }

    return 0;
}