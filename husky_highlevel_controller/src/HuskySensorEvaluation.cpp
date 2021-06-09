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
            ROS_INFO("Cancelling HuskySensorEvaluation node.");
        }
        else {
            subscriber_             = nodeHandle_.subscribe(scanTopic_, queueSize_, &HuskySensorEvaluation::topicCallback, this);
            scanPublisher_          = nodeHandle_.advertise<sensor_msgs::LaserScan>(scanPublisherTopic_, queueSize_);
            huskyMovePublisher_     = nodeHandle_.advertise<husky_highlevel_controller_msgs::HuskyMove>(huskyMovePublisherTopic_, queueSize_);
            read_parameterservice_  = nodeHandle_.advertiseService("read_parameters", &HuskySensorEvaluation::readParamCallback, this);
            ROS_INFO("Successfully launched HuskySensorEvaluation node.");
        }
    }

    HuskySensorEvaluation::~HuskySensorEvaluation() {ROS_WARN("action server tot");}

    bool HuskySensorEvaluation::readParameters()
    {
        if (!nodeHandle_.getParam("scanTopic",                  scanTopic_))                { return false; }
        if (!nodeHandle_.getParam("queueSize",                  queueSize_))                { return false; }
        if (!nodeHandle_.getParam("scanPublisherTopic",         scanPublisherTopic_))       { return false; }
        if (!nodeHandle_.getParam("huskyMovePublisherTopic",    huskyMovePublisherTopic_))  { return false; }
        if (!nodeHandle_.getParam("distanceToWall",             distanceToWall_))           { return false; }
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

        //ROS_INFO_STREAM("Minimum Distance of Laserscan: " << minDistance);   
        
        publishRecreatedScan(message, searchedIdx);
        publishHuskyMove(message);
    }

    void HuskySensorEvaluation::publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        sensor_msgs::LaserScan recreatedScan = algorithm_.GetRecreatedScan(message, searchedIdx);
        scanPublisher_.publish(recreatedScan);        
    }

    void HuskySensorEvaluation::publishHuskyMove(const sensor_msgs::LaserScan& message)
    {
        husky_highlevel_controller_msgs::HuskyMove huskyMove = algorithm_.WallFollower(message, distanceToWall_);
        huskyMovePublisher_.publish(huskyMove);        
    }
}
