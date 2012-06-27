#include "ros/ros.h"

#include "accompany_example_action/example_action_client.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "action_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	ExampleActionClient client(n);

	// only proceed if the action server is available
	bool serverAvailable = client.init();
	if (serverAvailable == false)
	{
		std::cout << "The connection to the action server could not be established.\n" << std::endl;
		return -1;
	}
	std::cout << "The action server was found.\n" << std::endl;

	// start the interactive client
	client.run();

	return 0;
}
