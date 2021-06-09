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
    std::tuple<float, float> Algorithm::GetScanCoordinates(const sensor_msgs::LaserScan& currentScan, int index)
    {
        float scanX, scanY;
        float angle, range;

        angle = currentScan.angle_min + index * currentScan.angle_increment;
        range = currentScan.ranges[index];

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

    /*!
    * Algorithm for getting the recreated scan
    * @param transformation current scan
    */
    husky_highlevel_controller_msgs::HuskyMove Algorithm::WallFollower(const sensor_msgs::LaserScan& scan, float distanceToObject)
    {        
        sensor_msgs::LaserScan laserMsg = scan;
        // declare an array of float values to keep track of the laser measurements
        std::vector<float> ranges = laserMsg.ranges;
        size_t range_size = ranges.size();
        ROS_INFO_ONCE("Number of laser rays: [%zu]", range_size); // for debugging
        // 720 individual laser rays published
        // variables to store closest (min) distance values on each zone
        // array to keep track of the min. distance value on each zone
        float minDis[4];
        minDis[0] = laserMsg.range_max;
        minDis[1] = laserMsg.range_max;
        minDis[2] = laserMsg.range_max;
        minDis[3] = laserMsg.range_max;
        minDis[4] = laserMsg.range_max;

        float range_max = laserMsg.range_min;
        // cycle trough all laser range rays
        for (size_t i = 0; i < range_size; i++) 
        {
            // rays < 144, laser rays to the far right side
            if (i < range_size / 5) {
                // get the smallest (closest) laser range value
                if (ranges[i] < minDis[0]) {
                  minDis[0] = ranges[i];
                }
            }
            // rays >= 144 and rays < 288, laser rays to the front-right side
            else if (i >= range_size / 5 && i < range_size * 2 / 5) {
                // get the smallest (closest) laser range value
                if (ranges[i] < minDis[1]) {
                  minDis[1] = ranges[i];
                }
            }
            // rays >= 288 and rays < 432, laser rays to the front
            else if (i >= range_size * 2 / 5 && i < range_size * 3 / 5) {
                // get the smallest (closest) laser range value
                if (ranges[i] < minDis[2]) {
                  minDis[2] = ranges[i];
                }
            }
            // rays >= 432 and rays < 576, laser rays to the front-left side
            else if (i >= range_size * 3 / 5 && i < range_size * 4 / 5) {
                // get the smallest (closest) laser range value
                if (ranges[i] < minDis[3]) {
                  minDis[3] = ranges[i];
                }
            }
            // rays > 576 and rays <= 720, laser rays to the far left side
            else if (i >= range_size * 4 / 5 && i <= range_size) {
                // get the smallest (closest) laser range value
                if (ranges[i] < minDis[4]) {
                  minDis[4] = ranges[i];
                }
            } else {
                ROS_ERROR("Ray index not found in range size");
            }
        } // end of for
        // ROS_INFO("Closest object to the far right: [%f]: ", minDis[0]);
        // ROS_INFO("Closest object to the front-right: [%f]: ", minDis[1]);
        // ROS_INFO("Closest object to the front: [%f]: ", minDis[2]);
        // ROS_INFO("Closest object to the front-left: [%f]: ", minDis[3]);
        // ROS_INFO("Closest object to the far left: [%f]: ", minDis[4]);


        // fine tune distance (mt) used to consider a region as blocked by an obstacle
        float d = distanceToObject;
        int moveCmd;
        husky_highlevel_controller_msgs::HuskyMove moveMsg;
        moveMsg.header.frame_id = "base_laser";
        moveMsg.header.stamp    = ros::Time();

        // logic block 1:
        if (minDis[0] > d && minDis[1] > d && minDis[2] > d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 1: no obstacles detected");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] < d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 2: obstacle only in front zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] > d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 3: obstacle only in front-right zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] > d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 4: obstacle only in front-left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] < d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 5: obstacle in front-right and front zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] < d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 6: obstacle in front and front-left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] < d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 7: obstacle in front-right, front and front-left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] > d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 8: obstacle in front-right and front-left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        }
        // logic block 2:
        else if (minDis[0] < d && minDis[1] > d && minDis[2] > d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 9:  obstacle only in right zone");
            moveCmd = FOLLOW_WALL; // follow the wall: keep moving straight ahead
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] < d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 10:  obstacle in right and front zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] > d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 11: obstacle in right and front-right zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] > d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 12: obstacle in right and front-left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] < d && minDis[3] > d && minDis[4] > d) {
            ROS_INFO("case 13: obstacle in right, front-right and front zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] < d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 14: obstacle in right, front and front-left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] < d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 15: obst. in right, front-right, front and front-left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] > d && minDis[3] < d && minDis[4] > d) {
            ROS_INFO("case 16: obstacle in right, front-right and front-left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        }
        // logic block 3:
        else if (minDis[0] > d && minDis[1] > d && minDis[2] > d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 17: obstacle only in left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] < d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 18: obstacle in front and left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] > d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 19: obstacle in front-right and left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] > d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 20: obstacle in front-left and left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] < d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 21: obstacle in front-right, front and left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] > d && minDis[1] > d && minDis[2] < d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 22: obstacle in front, front-left and left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] < d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 23: obst. in front-right, front, front-left and left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] > d && minDis[1] < d && minDis[2] > d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 24: obstacle in front-right, front-left and left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead 
        }
        // logic block 4:
        else if (minDis[0] < d && minDis[1] > d && minDis[2] > d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 25: obstacle in right and left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] < d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 26: obstacle in right, front and left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] > d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 27: obstacle in right, front-right and left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] > d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 28: obstacle in right, front-left and left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] < d && minDis[3] > d && minDis[4] < d) {
            ROS_INFO("case 29: obstacle in right, front-right, front and left zone");
            moveCmd = TURN_LEFT; // turn left
        } else if (minDis[0] < d && minDis[1] > d && minDis[2] < d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 30: obstacle in right, front, front-left, and left zone");
            moveCmd = STRAIGHT; // find wall: turn CW and move ahead
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] < d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 31: obst. in right, front-right, front, front-left and left zone");
            moveCmd = REVERSE_LEFT; // reverse turning left
        } else if (minDis[0] < d && minDis[1] < d && minDis[2] > d && minDis[3] < d && minDis[4] < d) {
            ROS_INFO("case 32: obst. in right, front-right, front-left and left zone");
            moveCmd = STRAIGHT_SLOW; // move slow straight ahead
        } else {
            ROS_INFO("Unknown case");
        }
        moveMsg.moveCmd = moveCmd;

        return moveMsg;
    }    
}
