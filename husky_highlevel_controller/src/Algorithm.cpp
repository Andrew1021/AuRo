#include "husky_highlevel_controller/Algorithm.hpp"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

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
        for(int i = 1; i < queueSize; i++) {                    
            if (laserScan.ranges[i] < laserScan.ranges[neededIndex]) { 
                neededIndex = i; 
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
    std::tuple<float, float> GetScanAngleRange(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist)
    {
        float angle, range;

        angle = currentScan.angle_min + idxOfSmallestDist * currentScan.angle_increment;
        range = currentScan.ranges[idxOfSmallestDist];

        std::tuple<float, float> result = std::make_tuple(angle, range);
        return result;
    }

    /*!
    * Algorithm for getting the recreated scan
    * @param angle current angle 
    * @param range current range
    * @param kP    control param of P controller 
    */
    geometry_msgs::Twist ComputeCmdVelParam(const geometry_msgs::TransformStamped& transformation, float angle, float range, int kP)
    {
        geometry_msgs::Twist velMsg; // initialized with zeros

        float scanX = cosf(angle) * range;
        float scanY = sinf(angle) * range;

        

        return velMsg;
    }
}
