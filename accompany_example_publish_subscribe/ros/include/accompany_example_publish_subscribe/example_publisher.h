#include "ros/ros.h"
#include "std_msgs/String.h"

class Example_publisher
{
public:
	Example_publisher(ros::NodeHandle nh);
	void init();
	void do_publish();

protected:

	ros::NodeHandle node_;
	ros::Publisher string_publisher_;
};
