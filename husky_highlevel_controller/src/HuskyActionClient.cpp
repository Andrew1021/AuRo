#include "husky_highlevel_controller/HuskyActionClient.hpp"

namespace husky_highlevel_controller
{

    HuskyActionClient::HuskyActionClient(ros::NodeHandle &nh)
        : nh_(nh), ac_("husky_motion_action")
    {
        if (!readParameters())
        {
            ROS_ERROR("Coudl not read parameters!");
            ros::requestShutdown();
        }

        start_drive_service_ = nh_.advertiseService("start_drive", &HuskyActionClient::startDriveCB, this);
        stop_drive_service_ = nh_.advertiseService("stop_drive", &HuskyActionClient::stopDriveCB, this);

        ROS_INFO("Started husky_highlevel_contoller_client!");
    }

    HuskyActionClient::~HuskyActionClient()
    {
    }

    bool HuskyActionClient::readParameters()
    {
        if (!nh_.getParam("distanceToWall", _distanceToWall))
            return false;
        return true;
    }

    bool HuskyActionClient::startDriveCB(std_srvs::Empty::Request &req,
                                                      std_srvs::Empty::Response &res)
    {
        ROS_INFO("START DRIVE");
        husky_highlevel_controller::HuskyMotionControllerGoal goal;
        goal.min_y = _distanceToWall;
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
        ROS_INFO("Server State went to active");
    }

    void HuskyActionClient::serverDoneCB(const actionlib::SimpleClientGoalState &state,
                                                      const husky_highlevel_controller::HuskyMotionControllerResultConstPtr &result)
    {
        ROS_INFO("Server ended in state [%s]", state.toString().c_str());
    }

    void HuskyActionClient::serverFeedbackCB(const husky_highlevel_controller::HuskyMotionControllerFeedbackConstPtr &feedback)
    {
        ROS_INFO("Server Feedback reached. Distance to pillar left: [%.2f]", feedback->y);
    }

}