#include "ros/ros.h"
#include "std_msgs/String.h"

class Example_subscriber
{
public:
	Example_subscriber(ros::NodeHandle nh);
	void init();
	void receiveCallback(const std_msgs::String::ConstPtr& msg);

protected:

	ros::NodeHandle node_;
	ros::Subscriber string_subscriber_;
};
