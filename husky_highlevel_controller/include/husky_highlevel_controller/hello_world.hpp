#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace hello_world_controller {

/*!
 * Class containing the Talker Controller
 */
class HelloWorldController {
public:
	/*!
	 * Constructor.
	 */
	HelloWorldController(ros::NodeHandle & nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HelloWorldController();

private:
    //! ROS node handle.
	ros::NodeHandle nodeHandle_;

    //! ROS topic publish counter (default 0).
    unsigned int count_ = 0;

	/*!
    * ROS hello world method.
    */
    void printHello();

	/*!
    * ROS hello world method.
    * @param count the send counter.
    */
    void printHello(unsigned int count);
};
} /* namespace */