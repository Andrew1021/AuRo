#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>

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
            subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighLevelController::topicCallback, this);
            scanPublisher_  = nodeHandle_.advertise<geometry_msgs::Twist>(scanPublisherTopic_, queueSize_);
            cmdVelPublisher_= nodeHandle.advertise<geometry_msgs::Twist>(cmdVelPublisherTopic_, queueSize_);

            // First, advertise on the visualization_marker topic:
            vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

            read_parameter_service_ = nodeHandle_.advertiseService("read_parameters", &HuskyHighLevelController::readParametersServiceCB, this);

            ROS_INFO("Successfully started node.");
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
        return true;
    }

    bool HuskyHighLevelController::readParametersServiceCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Reading parameters from param server.");
        return readParameters();
    }

    void HuskyHighLevelController::topicCallback(const sensor_msgs::LaserScan& message)
    {
        float minDistance;
        int searchedIdx;

        std::tie(minDistance, searchedIdx) = algorithm_.GetMinDistance(message);

        ROS_INFO("Minimum Distance of Laserscan: %.4f.\n", minDistance);               
        
        publishRecreatedScan(message, searchedIdx);
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

    void HuskyHighLevelController::publishVisMarker(const double x, const double y)
    {
        // After that it's as simple as filling out a visualization_msgs/Marker message and publishing it:
        geometry_msgs::TransformStamped transform_stamped; 
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_laser";
        marker.header.stamp = ros::Time();
        marker.ns = "husky_highlevel_controller";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE; //CYLINDER; //SPHERE; //ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = -M_PI_2; //0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0; // frist red
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        //only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        // Publish marker with red color
        vis_pub_.publish(marker);

        // Change color to green for new marker
        marker.id = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        try
        {
            transform_stamped = tfBuffer_.lookupTransform("odom", marker.header.frame_id, ros::Time(0));
            // TODO: calculations pose

            //marker.pose = pose;
            marker.header.frame_id = "odom";
            // publish marker
            vis_pub_.publish( marker );

        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}// namespace husky_highlevel_controller