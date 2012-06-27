#include "ros/ros.h"
#include "std_msgs/String.h"

class ExamplePublisher
{
public:
	ExamplePublisher(ros::NodeHandle nh);
	void init();
	void do_publish();

protected:

	ros::NodeHandle node_;
	ros::Publisher string_publisher_;
};
