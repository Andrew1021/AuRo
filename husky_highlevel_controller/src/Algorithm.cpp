#include "husky_highlevel_controller/Algorithm.hpp"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>

namespace husky_highlevel_controller 
{    
    Algorithm::Algorithm() {}

    /*!
    * Destructor.
    */
    Algorithm::~Algorithm() = default;

    /*!
    * Algorithm for getting the minimum distance
    * @param currentScan the received message.
    * @return 1. float mimmum distance; 2. int index
    */
    std::tuple<float, int> Algorithm::GetMinDistance(const sensor_msgs::LaserScan& currentScan)
    {
        std::tuple<float, int> result; 
        sensor_msgs::LaserScan laserScan = currentScan;
        int queueSize = laserScan.ranges.size();

        int neededIndex = 0;
        for(int j = 1; j < queueSize; j++) {                    
            if (laserScan.ranges[j] < laserScan.ranges[neededIndex]) { 
                neededIndex = j; 
            }
        }

        result = std::make_tuple(laserScan.ranges[neededIndex], neededIndex);
        return result;
    }

    /*!
    * Algorithm for getting the recreated scan
    * @param currentScan the received message
    * @param idxOfSmallestDist, index of smallest distance, queueSize
    * @return recreatedScan
    */
    sensor_msgs::LaserScan Algorithm::GetRecreatedScan(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist)
    {
        sensor_msgs::LaserScan recreatedScan = currentScan;
          
        // Check, wheter the two values before and afterwards 
        // are in the range of the basic array
        const int size = currentScan.ranges.size();
        std::vector<float> ranges;
        for(int j = idxOfSmallestDist-2; j <= idxOfSmallestDist+2; j++)
        {
            bool inRange = (0 <= j) && (j < size);
            if(inRange) {
                ranges.push_back(currentScan.ranges[j]);
            }
        }
        recreatedScan.ranges = ranges;
        recreatedScan.angle_min = currentScan.angle_min + (idxOfSmallestDist - recreatedScan.ranges.size() * 0.5) * recreatedScan.angle_increment;
        recreatedScan.angle_max = currentScan.angle_min + (idxOfSmallestDist + recreatedScan.ranges.size() * 0.5) * recreatedScan.angle_increment;

        if(recreatedScan.angle_max > currentScan.angle_max) {
            recreatedScan.angle_max = currentScan.angle_max;
        }
        if(recreatedScan.angle_min < currentScan.angle_min) {
            recreatedScan.angle_min = currentScan.angle_min;
        }

        return recreatedScan;
    }

    /*!
    * Algorithm for getting the recreated scan
    * @param currentScan the received message
    * @param idxOfSmallestDist, index of smallest distance, queueSize
    * @return 1. float angle; 2. float range
    */
    std::tuple<float, float> Algorithm::GetScanCoordinates(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist)
    {
        float scanX, scanY;
        float angle, range;

        angle = currentScan.angle_min + idxOfSmallestDist * currentScan.angle_increment;
        range = currentScan.ranges[idxOfSmallestDist];

        scanX = cosf(angle) * range;
        scanY = sinf(angle) * range;

        std::tuple<float, float> result = std::make_tuple(scanX, scanY);
        return result;
    }

    /*!
    * Algorithm for getting the recreated scan
    * @param transformation current transformation
    * @param angle current angle 
    * @param range current range
    * @param kP    control param of P controller 
    */
    geometry_msgs::Twist Algorithm::ComputeCmdVelParam(const geometry_msgs::TransformStamped& transformation, float scanX, float scanY, float kP)
    {
        geometry_msgs::Twist velMsg; // initialized with zeros

        geometry_msgs::Pose pose;
        pose.position.x = scanX;
        pose.position.y = scanY;
        ROS_INFO_STREAM("pose before: " << pose);
        tf2::doTransform(pose, pose, transformation);
        ROS_INFO_STREAM("pose after: " << pose);

        velMsg.linear.x     = kP * pose.position.x;
        velMsg.linear.y     = kP * pose.position.y;
        velMsg.angular.z    = kP * atan2(pose.position.y, pose.position.x); //recreated angle from new frames
        
        ROS_INFO_STREAM("velMsg: " << velMsg);

        return velMsg;
    }
}
