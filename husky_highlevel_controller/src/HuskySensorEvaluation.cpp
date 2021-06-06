#include "husky_highlevel_controller/HuskySensorEvaluation.hpp"

// STD
#include <string>

namespace husky_highlevel_controller
{
    HuskySensorEvaluation::HuskySensorEvaluation(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
            ROS_INFO("Cancelling scan node.");
        }
        else {
            subscriber_     = nodeHandle_.subscribe(scanTopic_, queueSize_, &HuskySensorEvaluation::topicCallback, this);
            scanPublisher_  = nodeHandle_.advertise<sensor_msgs::LaserScan>(scanPublisherTopic_, queueSize_);
            read_parameterservice_ = nodeHandle_.advertiseService("read_parameters", &HuskySensorEvaluation::readParamCallback, this);
            ROS_INFO("Successfully launched node.");
        }
    }

    HuskySensorEvaluation::~HuskySensorEvaluation() {}

    bool HuskySensorEvaluation::readParameters()
    {
        if (!nodeHandle_.getParam("scanTopic",          scanTopic_))      { return false; }
        if (!nodeHandle_.getParam("queueSize",          queueSize_))            { return false; }
        if (!nodeHandle_.getParam("scanPublisherTopic", scanPublisherTopic_))   { return false; }
        return true;
    }

    bool HuskySensorEvaluation::readParamCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Reading parameter from parameter server...");
        return readParameters();
    }

    void HuskySensorEvaluation::topicCallback(const sensor_msgs::LaserScan& message)
    {
        float minDistance;
        int searchedIdx;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);

        ROS_INFO_STREAM("Minimum Distance of Laserscan: " << minDistance);   
        
        publishRecreatedScan(message, searchedIdx);
    }

    void HuskySensorEvaluation::publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        sensor_msgs::LaserScan recreatedScan = algorithm_.GetRecreatedScan(message, searchedIdx);

        scanPublisher_.publish(recreatedScan);        
    }
}