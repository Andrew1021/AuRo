#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "husky_highlevel_controller_msgs/HuskyMotionControllerAction.h"
#include "husky_highlevel_controller_msgs/Cancel.h"
#include "husky_highlevel_controller_msgs/HuskyMove.h"

namespace husky_highlevel_controller
{
    class HuskyActionClient
    {
        public: 
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            HuskyActionClient(ros::NodeHandle& nodeHandle);

            /*!
            * Destructor.
            */
            virtual ~HuskyActionClient();

        private:
            // ROS node handle
            ros::NodeHandle nh_;

            actionlib::SimpleActionClient<husky_highlevel_controller_msgs::HuskyMotionControllerAction> ac_;

            // ROS Service to re-read parameters
            ros::ServiceServer read_parameter_server_service_;

            husky_highlevel_controller_msgs::HuskyMove prev_;

            // cancels goal
            bool ac_move_cancel_;

            //! ROS SUBSCRIBER
            //! ROS topic subscriber.
            ros::Subscriber subscriber_;
            //! ROS topic name to subscribe to.
            std::string cancelTopic_;
            //! Queue size of LaserScan message
            int queueSize_;

            bool readParameters();

            bool readParametersServiceCB(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res);

            /*!
            * ROS topic callback method.
            * @param message the received message.
            */
            void topicCallback(const husky_highlevel_controller_msgs::Cancel& cancel_msg);

            /**
             * @brief Service Callback to start drive.
             */
            void startDriveCB();

            /**
             * @brief Service Callback to stop drive.
             */
            void stopDriveCB();

            /**
             * @brief Server Active Callback Method.
             * 
             */
            void serverActiveCB();

            /**
             * @brief Callback method will be called when action server is in a done state.
             * 
             * @param state Action State
             * @param result Result
             */
            void serverDoneCB(const actionlib::SimpleClientGoalState &state,
                            const husky_highlevel_controller_msgs::HuskyMotionControllerResultConstPtr &result);

            /**
             * @brief Callback method will be called if a feedback from server is available.
             * 
             * @param feedback feedback.
             */
            void serverFeedbackCB(const husky_highlevel_controller_msgs::HuskyMotionControllerFeedbackConstPtr &feedback);

    };
}