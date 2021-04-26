#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller 
{    
    Algorithm::Algorithm() {}

    /*!
    * Destructor.
    */
    Algorithm::~Algorithm() = default;

    /*!
    * Algorithm for getting the recreated scan
    * @param message the received message.
    */
    std::tuple<float, int> Algorithm::GetMinDistance(const sensor_msgs::LaserScan& currentScan)
    {
        std::tuple<float, int> result; 
        sensor_msgs::LaserScan laserScan = currentScan;
        int queueSize = laserScan.ranges.size();

        // discard unnecessary data
        for(int i = 0; i < queueSize; i++) {
            bool toDiscard = (laserScan.ranges[i] < laserScan.range_min) || (laserScan.range_max < laserScan.ranges[i]);
            if(toDiscard) { laserScan.ranges.erase(laserScan.ranges.begin()+i); }
        }

        // Get the index of the smallest distance
        queueSize = laserScan.ranges.size();
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
    * @param message the received message.
    */
    sensor_msgs::LaserScan Algorithm::GetRecreatedScan(const sensor_msgs::LaserScan& currentScan, int idxOfSmallestDist, int queueSize)
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
        recreatedScan.angle_min = currentScan.angle_min + (idxOfSmallestDist - queueSize * 0.5) * recreatedScan.angle_increment;
        recreatedScan.angle_max = currentScan.angle_max + (idxOfSmallestDist + queueSize * 0.5) * recreatedScan.angle_increment;

        return recreatedScan;
    }
}
