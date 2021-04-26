#pragma once
#include <sensor_msgs/LaserScan.h>

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
            * Algorithm for getting the recreated scan
            * @param message the received message.
            */
            std::tuple<float, int> GetMinDistance(const sensor_msgs::LaserScan& currentScan);

            /*!
            * Algorithm for getting the recreated scan
            * @param message the received message.
            */
            sensor_msgs::LaserScan GetRecreatedScan(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist, int queueSize);
    };
}
