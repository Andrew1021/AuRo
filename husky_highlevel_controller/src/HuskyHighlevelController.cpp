#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <geometry_msgs/Twist.h>

// STD
#include <string>

namespace husky_highlevel_controller
{
    HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
            ROS_INFO("Cancelling scan node.");
        }
        else {
            subscriber_     = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighLevelController::topicCallback, this);
            scanPublisher_  = nodeHandle_.advertise<sensor_msgs::LaserScan>(scanPublisherTopic_, queueSize_);
            cmdVelPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelPublisherTopic_, queueSize_);
            ROS_INFO("Successfully launched node.");
        }
    }

    HuskyHighLevelController::~HuskyHighLevelController() {}

    bool HuskyHighLevelController::readParameters()
    {
        if (!nodeHandle_.getParam("subscriberTopic",    subscriberTopic_))      { return false; }
        if (!nodeHandle_.getParam("queueSize",          queueSize_))            { return false; }
        if (!nodeHandle_.getParam("scanPublisherTopic", scanPublisherTopic_))   { return false; }
        if (!nodeHandle_.getParam("velPublisherTopic",  cmdVelPublisherTopic_)) { return false; }
        if (!nodeHandle_.getParam("kP",                 kP_))                   { return false; }
        if (!nodeHandle_.getParam("collisionThreshold", collisionThreshold_))   { return false; }
        return true;
    }

    void HuskyHighLevelController::topicCallback(const sensor_msgs::LaserScan& message)
    {
        float minDistance;
        int searchedIdx;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);

        ROS_INFO_STREAM("Minimum Distance of Laserscan: " << minDistance);   
        
        publishRecreatedScan(message, searchedIdx);

        nodeHandle_.getParam("kP", kP_);          
        nodeHandle_.getParam("collisionThreshold", collisionThreshold_);  
        if(minDistance > (collisionThreshold_ * kP_)) {
            navigateToPillar(message, searchedIdx);
        }
    }

    void HuskyHighLevelController::publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        sensor_msgs::LaserScan recreatedScan = algorithm_.GetRecreatedScan(message, searchedIdx);

        scanPublisher_.publish(recreatedScan);        
    }

    void HuskyHighLevelController::navigateToPillar(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        float scanAngle, scanRange;
        std::tie(scanAngle, scanRange) = algorithm_.GetScanAngleRange(message, searchedIdx);

        geometry_msgs::TransformStamped transformation;
        try {
            transformation = tfBuffer_.lookupTransform("base_link", "base_laser", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        geometry_msgs::Twist cmdVelParam = algorithm_.ComputeCmdVelParam(transformation, scanAngle, scanRange, kP_);

        cmdVelPublisher_.publish(cmdVelParam);
    }

} /* namespace */
