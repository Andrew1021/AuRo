#pragma once

#include "husky_highlevel_controller_msgs/HuskyMove.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
namespace husky_highlevel_controller 
{
    typedef enum _ROBOT_MOVEMENT {
        STRAIGHT = 0,
        TURN_LEFT,
        FOLLOW_WALL,
        STRAIGHT_SLOW,
        REVERSE_LEFT,
        STOP
    } ROBOT_MOVEMENT;       
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
            
            // Define the robot direction of movement  

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
            * @param scan  current laserScan 
            */
            husky_highlevel_controller_msgs::HuskyMove WallFollower(const sensor_msgs::LaserScan& scan, float distanceToObject);
    };
}
