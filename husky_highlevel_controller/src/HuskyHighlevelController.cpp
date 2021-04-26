#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

// STD
#include <string>

namespace husky_highlevel_controller
{
    HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
            ROS_INFO("Cancelling scan node.");
        }
        else {
            subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1, &HuskyHighLevelController::topicCallback, this);
            publisher_  = nodeHandle_.advertise<sensor_msgs::LaserScan>(publisherTopic_, 10);
            ROS_INFO("Successfully launched scan node.");
        }
    }

    HuskyHighLevelController::~HuskyHighLevelController() {}

    bool HuskyHighLevelController::readParameters()
    {
        if (!nodeHandle_.getParam("subscriberTopic",    subscriberTopic_))  { return false; }
        if (!nodeHandle_.getParam("queueSize",          queueSize_))        { return false; }
        if (!nodeHandle_.getParam("publisherTopic",     publisherTopic_))   { return false; }
        return true;
    }

    void HuskyHighLevelController::topicCallback(const sensor_msgs::LaserScan& message)
    {
        float minDistance;
        int searchedIdx;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);
        //* For part 5:*/ ROS_INFO_STREAM("min Laserscan: " << minDistance);
        
        publishRecreatedScan(message, searchedIdx);
    }

    void HuskyHighLevelController::publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        sensor_msgs::LaserScan recreatedScan = algorithm_.GetRecreatedScan(message, searchedIdx, queueSize_);

        publisher_.publish(recreatedScan);        
    }

} /* namespace */