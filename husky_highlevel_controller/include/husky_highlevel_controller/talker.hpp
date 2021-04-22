#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace talker_controller {

/*!
 * Class containing the Talker Controller
 */
class TalkerController {
public:
	/*!
	 * Constructor.
	 */
	TalkerController(ros::NodeHandle & nodeHandle, ros::Publisher chatterPublisher);

	/*!
	 * Destructor.
	 */
	virtual ~TalkerController();

private:
    //! ROS node handle.
	ros::NodeHandle nodeHandle_;

    //! ROS topic publisher.
    ros::Publisher chatterPublisher_;

    //! ROS topic publish message.
    std_msgs::String message_;

    //! ROS topic publish counter (default 0).
    unsigned int count_ = 0;
};

} /* namespace */