#include "husky_highlevel_controller/HuskyMotionController.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

// STD
#include <string>

namespace husky_highlevel_controller
{
    HuskyMotionController::HuskyMotionController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), 
    tfListener_(tfBuffer_), as_("husky_motion_action", boost::bind(&HuskyMotionController::topicCallback, this, _1), false)
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
            ROS_INFO("Cancelling scan node.");
        }
        else {
            subscriber_     = nodeHandle_.subscribe(scanPublisherTopic_, queueSize_, &HuskyMotionController::topicCallback, this);
            cmdVelPublisher_= nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelPublisherTopic_, queueSize_);
            markerPublisher_= nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
            read_parameterservice_ = nodeHandle_.advertiseService("read_parameters", &HuskyMotionController::readParamCallback, this);
            as_.registerPreemptCallback(boost::bind(&HuskyMotionController::preemptCB, this));
            as_.start();
            ROS_INFO("Successfully launched node.");
        }
    }

    HuskyMotionController::~HuskyMotionController() 
    {
        as_.shutdown();
    }

    bool HuskyMotionController::readParameters()
    {
        if (!nodeHandle_.getParam("queueSize",          queueSize_))            { return false; }
        if (!nodeHandle_.getParam("scanPublisherTopic", scanPublisherTopic_))   { return false; }
        if (!nodeHandle_.getParam("velPublisherTopic",  cmdVelPublisherTopic_)) { return false; }
        if (!nodeHandle_.getParam("kP",                 kP_))                   { return false; }
        if (!nodeHandle_.getParam("collisionThreshold", collisionThreshold_))   { return false; }
        if (!nodeHandle_.getParam("visuMarker",         markerPublisherTopic_)) { return false; }
        return true;
    }

    bool HuskyMotionController::readParamCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Reading parameter from parameter server...");
        return readParameters();
    }

    void HuskyMotionController::topicCallback(const husky_highlevel_controller::HuskyMotionControllerGoalConstPtr &goal)
    {
        float minDistance;
        int searchedIdx;
        float min_angle = goal->min_x;     // angle
        float min_distance = goal->min_y;  // range

        sensor_msgs::LaserScan message;
        message.angle_min = min_angle;
        message.range_min = min_distance;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);

        //ROS_INFO_STREAM("Minimum Distance of Laserscan: " << minDistance);   

        navigateToPillar(message, searchedIdx);

        publishMarkerRviz(message, searchedIdx);
    }

    void HuskyMotionController::navigateToPillar(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        float scanX, scanY;
        std::tie(scanX, scanY) = algorithm_.GetScanCoordinates(message, searchedIdx);

        ROS_INFO_STREAM("Minimum Distance of Laserscan: " << scanY);  
        ROS_INFO_STREAM("Angle of Laserscan: " << scanX);  

        geometry_msgs::TransformStamped transformation;
        try {
            transformation = tfBuffer_.lookupTransform("base_link", "base_laser", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        geometry_msgs::Twist cmdVelParam = algorithm_.ComputeCmdVelParam(transformation, scanX, scanY, kP_);

        cmdVelPublisher_.publish(cmdVelParam);
        as_.setSucceeded(result_);

    }

    void HuskyMotionController::preemptCB()
    {
        ROS_WARN("Husky Motion Action: Preempted");

        geometry_msgs::Twist zero_vel;

        cmdVelPublisher_.publish(zero_vel);

        as_.setPreempted();
    }

    void HuskyMotionController::publishMarkerRviz(const sensor_msgs::LaserScan& message, int searchedIdx)
    {
        visualization_msgs::Marker marker_baselaser;
        visualization_msgs::Marker marker_odom;
        geometry_msgs::Pose pose;
        geometry_msgs::TransformStamped transformation;

        float scanX, scanY;
        std::tie(scanX, scanY) = algorithm_.GetScanCoordinates(message, searchedIdx);

        pose.position.x = scanX+0.5;
        pose.position.y = scanY;
        pose.position.z = -1.0;
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        // marker base_laser frame
        marker_baselaser.header.frame_id = "base_laser";
        marker_baselaser.header.stamp = ros::Time();
        marker_baselaser.id = 0;
        marker_baselaser.type = visualization_msgs::Marker::CYLINDER; 
        marker_baselaser.action = visualization_msgs::Marker::ADD;
        marker_baselaser.pose = pose;
        marker_baselaser.scale.x = 1.0;
        marker_baselaser.scale.y = 1.0;
        marker_baselaser.scale.z = 3.0;
        marker_baselaser.color.a = 1.0;
        marker_baselaser.color.r = 0.0;
        marker_baselaser.color.g = 1.0;
        marker_baselaser.color.b = 0.0;
        markerPublisher_.publish(marker_baselaser);

        // marker odom frame
        marker_odom = marker_baselaser;
        marker_odom.id = 1;
        marker_odom.color.r = 1.0;
        marker_odom.color.g = 0.0;
        marker_odom.color.b = 1.0;
        marker_odom.header.frame_id = "odom";

        // Transform pose from base_laser to odom
        try
        {
            transformation = tfBuffer_.lookupTransform("odom", "base_laser", ros::Time(0));
            tf2::doTransform(pose, pose, transformation);
            marker_odom.pose = pose;
            markerPublisher_.publish(marker_odom);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }   

} /* namespace */
