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

        ac_.waitForServer();
        ac_move_cancel_ = false;
        prev_.moveCmd   = STOP;
        subscriber_     = nh_.subscribe(cancelTopic_, queueSize_, &HuskyActionClient::topicCallback, this);

        HuskyActionClient::startDriveCB();

        ROS_INFO("Successfully launched HuskyActionClient node.");
    }

    HuskyActionClient::~HuskyActionClient()
    {
        HuskyActionClient::stopDriveCB();
    }

    bool HuskyActionClient::readParameters()
    {
        if (!nh_.getParam("acCancelTopic", cancelTopic_)) { return false; }
        return true;
    }

    void HuskyActionClient::topicCallback(const husky_highlevel_controller_msgs::Cancel& msg) 
    {
        ac_move_cancel_ = msg.cancel;
        startDriveCB();
    }

    void HuskyActionClient::startDriveCB()
    {
        husky_highlevel_controller_msgs::HuskyMotionControllerGoal goal;
        goal.moveEnabled = !ac_move_cancel_;
        ac_.sendGoal(goal, boost::bind(&HuskyActionClient::serverDoneCB, this, _1, _2),
                     boost::bind(&HuskyActionClient::serverActiveCB, this),
                     boost::bind(&HuskyActionClient::serverFeedbackCB, this, _1));
    }

    void HuskyActionClient::stopDriveCB()
    {
        ac_.cancelAllGoals();
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
            cmd = "TURN_LEFT";
            break;

            case FOLLOW_WALL:
            cmd = "FOLLOW_WALL";
            break;

            case STRAIGHT_SLOW:
            cmd = "STRAIGHT_SLOW";
            break;

            case REVERSE_LEFT:
            cmd = "REVERSE_LEFT";
            break;

            default:
            cmd = "STOP";
            break;
        }

        if(feedback->executedMoveCmd != prev_.moveCmd) {
            ROS_INFO_STREAM("Server Feedback received. Excuted Husky Move Command: " << cmd);
            prev_.moveCmd = feedback->executedMoveCmd;
        }
    }
}