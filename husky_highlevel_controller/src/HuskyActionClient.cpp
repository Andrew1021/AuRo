#include "husky_highlevel_controller/HuskyActionClient.hpp"
#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller
{
    HuskyActionClient::HuskyActionClient(ros::NodeHandle &nh) : nh_(nh), ac_("husky_motion_action")
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters!");
            ros::requestShutdown();
            ROS_ERROR("Cancelling HuskyMotionController node.");
        }

        subscriber_          = nh_.subscribe(cancelTopic_, queueSize_, &HuskyActionClient::topicCallback, this);
        start_drive_service_ = nh_.advertiseService("start_drive", &HuskyActionClient::startDriveCB, this);
        stop_drive_service_  = nh_.advertiseService("stop_drive", &HuskyActionClient::stopDriveCB, this);

        ROS_WARN("Started husky_action_client!");
    }

    HuskyActionClient::~HuskyActionClient()
    {
    }

    bool HuskyActionClient::readParameters()
    {
        if (!nh_.getParam("acCancelTopic", cancelTopic_)) { return false; }
        return true;
    }

    void HuskyActionClient::topicCallback(const husky_highlevel_controller_msgs::Cancel& msg) 
    {
        ac_move_cancel_ = msg.cancel;
    }

    bool HuskyActionClient::startDriveCB(std_srvs::Empty::Request &req,
                                                      std_srvs::Empty::Response &res)
    {
        ROS_INFO("START DRIVE");
        husky_highlevel_controller_msgs::HuskyMotionControllerGoal goal;
        goal.moveEnabled = !ac_move_cancel_;
        ac_.sendGoal(goal, boost::bind(&HuskyActionClient::serverDoneCB, this, _1, _2),
                     boost::bind(&HuskyActionClient::serverActiveCB, this),
                     boost::bind(&HuskyActionClient::serverFeedbackCB, this, _1));
        return true;
    }

    bool HuskyActionClient::stopDriveCB(std_srvs::Empty::Request &req,
                                                     std_srvs::Empty::Response &res)
    {
        ac_.cancelAllGoals();
        return true;
    }

    void HuskyActionClient::serverActiveCB()
    {
        ROS_WARN("Server State went to active");
    }

    void HuskyActionClient::serverDoneCB(const actionlib::SimpleClientGoalState &state,
                                                      const husky_highlevel_controller_msgs::HuskyMotionControllerResultConstPtr &result)
    {
        ROS_INFO("Server ended in state [%s]", state.toString().c_str());
    }

    void HuskyActionClient::serverFeedbackCB(const husky_highlevel_controller_msgs::HuskyMotionControllerFeedbackConstPtr &feedback)
    {
        std::string cmd;
        switch (feedback->executedMoveCmd) 
        {
            case STRAIGHT:
            cmd = "STRAIGHT";
            break;

            case TURN_LEFT:
            cmd = "STRAIGHT";
            break;

            case FOLLOW_WALL:
            cmd = "STRAIGHT";
            break;

            case STRAIGHT_SLOW:
            cmd = "STRAIGHT";
            break;

            case REVERSE_LEFT:
            cmd = "STRAIGHT";
            break;

            default:
            cmd = "STOP";
            break;
        }
        ROS_INFO_STREAM("Server Feedback received. Excuted Husky Move Command: " << cmd);
    }
}