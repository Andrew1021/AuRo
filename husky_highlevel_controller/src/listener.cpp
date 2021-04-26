#include <ros/ros.h>
#include <std_msgs/String.h>

#include "husky_highlevel_controller/listener.hpp"
#include "husky_highlevel_controller/Algorithm.hpp"

// STD
#include <string>

namespace listener_controller 
{

    ListenerController::ListenerController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        // recieve of topics with method from node handle
        // callback with data is started when message arrvies
        // stay connected with subscriber object until no message should be recieved
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                            &ListenerController::chatterCallback, this);

        serviceServer_ = nodeHandle_.advertiseService("get_min",
                                                        &ListenerController::serviceCallback, this);

        ROS_INFO("Successfully launched node.");
    }

    ListenerController::~ListenerController() {}

    bool ListenerController::readParameters()
    {
        if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_) &&
            !nodeHandle_.getParam("queue_size", queueSize_)) 
        {
            return false;
        }
        else
        {
            return true;
        }
        
    }

    void chatterCallback(const std_msgs::String & msg)
    {
        ROS_INFO("I heard: [%s]", msg.data.c_str());
    }

    void ListenerController::topicCallback(const sensor_msgs::LaserScan & message)
    {
        algorithm_.addData(message.range_min);
    }

    bool ListenerController::serviceCallback(std_srvs::Trigger::Request& request,
                                            std_srvs::Trigger::Response& response)
    {
        response.success = true;
        response.message = "The min Laserscan distance is " + std::to_string(algorithm_.getMin());
        return true;
    }

} /* namespace */