#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include "husky_highlevel_controller/Algorithm.hpp"

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

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    //! ROS topic name to subscribe to.
    std::int32_t queueSize_;

    //! ROS service server.
    ros::ServiceServer serviceServer_;

    //! Algorithm computation object.
    Algorithm algorithm_;

    /*!
    * ROS topic callback method.
    * @param msg the received message.
    */
    void chatterCallback(const std_msgs::String & msg);

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
     bool readParameters();

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
     void topicCallback(const sensor_msgs::LaserScan & message);

    /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
     bool serviceCallback(std_srvs::Trigger::Request& request,
                        std_srvs::Trigger::Response& response);



};

} /* namespace */