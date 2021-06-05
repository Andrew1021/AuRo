#include "husky_highlevel_controller/husky_highlevel_controller_subscriber.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

// STD
#include <string>

namespace husky_highlevel_controller_subscriber
{
    HuskyHighLevelControllerSubscriber::HuskyHighLevelControllerSubscriber(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
            ROS_INFO("Cancelling scan node.");
        }
        else {
            subscriber_     = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighLevelControllerSubscriber::topicCallback, this);
            scanPublisher_  = nodeHandle_.advertise<sensor_msgs::LaserScan>(scanPublisherTopic_, queueSize_);
            read_parameterservice_ = nodeHandle_.advertiseService("read_parameters", &HuskyHighLevelControllerSubscriber::readParamCallback, this);
            ROS_INFO("Successfully launched node.");
        }
    }

    HuskyHighLevelControllerSubscriber::~HuskyHighLevelControllerSubscriber() {}

    bool HuskyHighLevelControllerSubscriber::readParameters()
    {
        if (!nodeHandle_.getParam("subscriberTopic",    subscriberTopic_))      { return false; }
        if (!nodeHandle_.getParam("queueSize",          queueSize_))            { return false; }
        if (!nodeHandle_.getParam("scanPublisherTopic", scanPublisherTopic_))   { return false; }
        if (!nodeHandle_.getParam("kP",                 kP_))                   { return false; }
        if (!nodeHandle_.getParam("collisionThreshold", collisionThreshold_))   { return false; }
        return true;
    }

    bool HuskyHighLevelControllerSubscriber::readParamCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Reading parameter from parameter server...");
        return readParameters();
    }

    void HuskyHighLevelControllerSubscriber::topicCallback(const sensor_msgs::LaserScan& message)
    {
        float minDistance;
        int searchedIdx;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);

        ROS_INFO_STREAM("Minimum Distance of Laserscan: " << minDistance);   
        
        // nodeHandle_.getParam("kP", kP_);          
        // nodeHandle_.getParam("collisionThreshold", collisionThreshold_);  
        if(minDistance > (collisionThreshold_ * kP_)) {
            publishRecreatedScan(message, searchedIdx);
        }
    }

    void HuskyHighLevelControllerSubscriber::publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        sensor_msgs::LaserScan recreatedScan = algorithm_.GetRecreatedScan(message, searchedIdx);

        scanPublisher_.publish(recreatedScan);        
    }
}