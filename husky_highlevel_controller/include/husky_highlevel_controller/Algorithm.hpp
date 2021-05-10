#pragma once
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

namespace husky_highlevel_controller 
{
    class Algorithm
    {
        public:
            /*!
            * Constructor.
            */
            Algorithm();

            /*!
            * Destructor.
            */
            virtual ~Algorithm();            

            /*!
            * Algorithm for getting the minimum distance
            * @param currentScan the received message.
            */
            std::tuple<float, int> GetMinDistance(const sensor_msgs::LaserScan& currentScan);

            /*!
            * Algorithm for getting the recreated scan
            * @param currentScan the received message
            * @param idxOfSmallestDist, index of smallest distance, queueSize
            */
            sensor_msgs::LaserScan GetRecreatedScan(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist);

            /*!
            * Algorithm for getting the recreated scan
            * @param currentScan the received message
            * @param idxOfSmallestDist, index of smallest distance, queueSize
            */
            std::tuple<float, float> GetScanAngleRange(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist);

            /*!
            * Algorithm for getting the recreated scan
            * @param angle current angle 
            * @param range current range
            */
            geometry_msgs::Twist ComputeCmdVelParam(const geometry_msgs::TransformStamped& transformation, float angle, float range, int kP);
    };
}
