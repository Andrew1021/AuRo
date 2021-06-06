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
            * @return 1. float scanX; 2. float scanY
            */
            std::tuple<float, float> GetScanCoordinates(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist);

            /*!
            * Algorithm for getting the recreated scan
            * @param transformation current transformation
            * @param angle current angle 
            * @param range current range
            * @param kP    control param of P controller 
            */
            geometry_msgs::Twist ComputeCmdVelParam(const geometry_msgs::TransformStamped& transformation, float angle, float range, float kP);

            /*!
            * Algorithm for getting the recreated scan
            * @param transformation current transformation
            * @param recreatedScan  recreated scan with the smallest distance and surrounding values 
            * @param kP             control param of P controller 
            */
            geometry_msgs::Twist WallFollowing(
                const geometry_msgs::TransformStamped& transformation, const sensor_msgs::LaserScan& recreatedScan, float kP
            );
    };
}
