#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "husky_highlevel_controller/Algorithm.hpp"
#include "husky_highlevel_controller_msgs/HuskyMove.h"

#include <actionlib/server/simple_action_server.h>
//#include <std_msgs/ColorRGBA.h>
#include <husky_highlevel_controller/HuskyMotionControllerAction.h>
//#include <husky_highlevel_controller/WallRoute.h>


namespace husky_highlevel_controller
{
    class HuskyMotionController 
    {   
        public: 
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            HuskyMotionController(ros::NodeHandle& nodeHandle);

            /*!
            * Destructor.
            */
            virtual ~HuskyMotionController();

            /**
             * @brief Execute Callback on new Goal
             * 
             * @param goal goal
             */
            void executeCB(const husky_highlevel_controller::HuskyMotionControllerGoalConstPtr &goal);

            /**
             * @brief Callback Method will be called on goal cancel.
             * 
             */
            void preemptCB();

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
            //void topicCallback(const sensor_msgs::LaserScan& message);
            void topicCallback(const husky_highlevel_controller_msgs::HuskyMove& message);

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
            * ROS publish method.
            * @param huskyMove the received message.
            * @param searchedIdx searched index
            */
            void FollowWall(const husky_highlevel_controller_msgs::HuskyMove& huskyMove);

            /*!
            * ROS publish method.
            * @param message the received message.
            * @param searchedIdx searched index
            */
            void publishMarkerRviz(const sensor_msgs::LaserScan& message);          

            /* @brief Service Callback for re-reading parameters
             *        from the ROS parameter server
             * 
             * @param req Empty request
             * @param res Empty response
             * @return true service finished successfully 
             * @return false service finished unsuccessfully
             */
            bool readParamCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

                                
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
            ros::Publisher cmdVelPublisher_;
            //! ROS topic name to publish to.
            std::string cmdVelPublisherTopic_;

            //! ROS PUBLISHER
            //! ROS topic publisher.
            ros::Publisher markerPublisher_;
            //! ROS topic name to publish to.
            std::string markerPublisherTopic_;

            husky_highlevel_controller_msgs::HuskyMove message_;

            std::string scanPublisherTopic_;

            ros::Subscriber subscriber_scan_;

            //! Control param of P-Controller
            float kP_;
            float collisionThreshold_;

            float _distance;

            //! TF Buffer
            tf2_ros::Buffer tfBuffer_;
            tf2_ros::TransformListener tfListener_;

            ros::ServiceServer read_parameterservice_;

            //! Algorithm computation object.
            Algorithm algorithm_;

            /**
             * @brief HuskyDrive Action Server
             * 
             * NOTE: NodeHandle instance must be created before this line. Otherwise strange error occurs.
             */
            actionlib::SimpleActionServer<husky_highlevel_controller::HuskyMotionControllerAction> as_;

            // HuskyDrive feedback
            husky_highlevel_controller::HuskyMotionControllerFeedback feedback_;

            // HuskyDriver result
            husky_highlevel_controller::HuskyMotionControllerResult result_;
    };
}
