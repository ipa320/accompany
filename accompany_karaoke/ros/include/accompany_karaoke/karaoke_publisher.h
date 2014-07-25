#include "ros/ros.h"
#include "std_msgs/String.h"
#include <SFML/Audio.hpp>

class KaraokePublisher
{
public:
	KaraokePublisher(ros::NodeHandle nh);
	void init();
	void do_publish();

protected:

	ros::NodeHandle node_;
	ros::Publisher string_publisher_;
};
