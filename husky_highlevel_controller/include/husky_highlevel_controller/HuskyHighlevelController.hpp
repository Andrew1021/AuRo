#pragma once

//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
    class HuskyHighLevelController 
    {   
        public: 
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            HuskyHighLevelController(ros::NodeHandle& nodeHandle);

            /*!
            * Destructor.
            */
            virtual ~HuskyHighLevelController();

        private:
            /*!
            * Reads and verifies the ROS parameters.
            * @return true if successful.
            */
            bool readParameters();

            /*!
            * ROS topic callback method.
            * @param message the received message.
            */
            void topicCallback(const sensor_msgs::LaserScan& message);

            /*!
            * ROS publish method.
            * @param message the received message.
            * @param searchedIdx searched index
            */
            void publishRecreatedScan(const sensor_msgs::LaserScan& message, int searchedIdx);

            /*!
            * ROS publish method.
            * @param message the received message.
            * @param searchedIdx searched index
            */
            void navigateToPillar(const sensor_msgs::LaserScan& message, int searchedIdx);

            /*!
            * ROS service server callback.
            * @param request the request of the service.
            * @param response the provided response.
            * @return true if successful, false otherwise.
            */
            //bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
                                
            //! ROS node handle.
            ros::NodeHandle& nodeHandle_;

            //! ROS SUBSCRIBER
            //! ROS topic subscriber.
            ros::Subscriber subscriber_;
            //! ROS topic name to subscribe to.
            std::string subscriberTopic_;
            //! Queue size of LaserScan message
            int queueSize_;

            //! ROS PUBLISHER
            //! ROS topic publisher.
            ros::Publisher scanPublisher_;
            //! ROS topic name to publish to.
            std::string scanPublisherTopic_;

            //! ROS PUBLISHER
            //! ROS topic publisher.
            ros::Publisher cmdVelPublisher_;
            //! ROS topic name to publish to.
            std::string cmdVelPublisherTopic_;

            //! Control param of P-Controller
            double kP_;

            //! TF Buffer
            tf2_ros::Buffer tfBuffer_;
            tf2_ros::TransformListener tfListener_;

            //! Algorithm computation object.
            Algorithm algorithm_;
    };
}