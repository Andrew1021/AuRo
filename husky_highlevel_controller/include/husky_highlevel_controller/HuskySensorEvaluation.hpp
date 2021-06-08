#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <husky_highlevel_controller/Algorithm.hpp>


namespace husky_highlevel_controller
{
    class HuskySensorEvaluation 
    {   
        public: 
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            HuskySensorEvaluation(ros::NodeHandle& nodeHandle);

            /*!
            * Destructor.
            */
            virtual ~HuskySensorEvaluation();

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
            */
            void publishHuskyMove(const sensor_msgs::LaserScan& message);    

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
            std::string scanTopic_;
            //! Queue size of LaserScan message
            int queueSize_;

            //! ROS PUBLISHER
            //! ROS topic publisher.
            ros::Publisher scanPublisher_;
            //! ROS topic name to publish to.
            std::string scanPublisherTopic_;

            //! ROS PUBLISHER
            //! ROS topic publisher.
            ros::Publisher huskyMovePublisher_;
            //! ROS topic name to publish to.
            std::string huskyMovePublisherTopic_;

            ros::ServiceServer read_parameterservice_;

            float distanceToWall_;

            //! Algorithm computation object.
            Algorithm algorithm_;
    };
}
