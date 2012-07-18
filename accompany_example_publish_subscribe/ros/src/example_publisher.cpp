#include "accompany_example_publish_subscribe/example_publisher.h"

#include <sstream>

ExamplePublisher::ExamplePublisher(ros::NodeHandle nh)
{
	node_ = nh;
}


void ExamplePublisher::init()
{
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The first parameter defines the data type of the messages that become published.
	* Here it is a string. Find all data types and instructions how to create
	* your own messages at ros.org.
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/

	string_publisher_ = node_.advertise<std_msgs::String>("example_publisher", 10);
}


void ExamplePublisher::do_publish()
{
	// publishing rate
	ros::Rate loop_rate(10);

	/**
	* A count of how many messages we have sent. This is used to create
	* a unique string for each message.
	*/
	int count = 0;

	// send messages with publishing rate
	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		string_publisher_.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
