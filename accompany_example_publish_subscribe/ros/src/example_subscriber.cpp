#include "accompany_example_publish_subscribe/example_subscriber.h"

#include <sstream>

ExampleSubscriber::ExampleSubscriber(ros::NodeHandle nh)
{
	node_ = nh;
}


void ExampleSubscriber::init()
{
	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called receiveCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	string_subscriber_ = node_.subscribe("example_publisher", 10, &ExampleSubscriber::receiveCallback, this);
}


void ExampleSubscriber::receiveCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
