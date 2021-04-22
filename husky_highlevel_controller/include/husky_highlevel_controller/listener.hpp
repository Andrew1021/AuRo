#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace listener_controller {

/*!
 * Class containing the Listner Controller
 */
class ListenerController {
public:
	/*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
	ListenerController(ros::NodeHandle & nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~ListenerController();


private:
    //! ROS node handle.
	ros::NodeHandle nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriber_;

    /*!
    * ROS topic callback method.
    * @param msg the received message.
    */
    void chatterCallback(const std_msgs::String & msg);

};

} /* namespace */