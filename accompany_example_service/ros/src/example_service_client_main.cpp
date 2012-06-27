#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <accompany_example_service/Message.h>

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
	ros::init(argc, argv, "service_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = ros::service::waitForService("square_number", 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service server is advertised.\n" << std::endl;

	// prepare the request and response messages
	accompany_example_service::Message::Request req;
	accompany_example_service::Message::Response res;

	while(1)
	{
		double num;
		std::cout << "Enter a number that shall be squared (enter 0 to quit): ";
		std::cin >> num;
		if (num == 0.0)
			return 0;
		req.number = num;

		// this calls the service server to process our request message and put the result into the response message
		// this call is blocking, i.e. this program will not proceed until the service server sends the response
		bool success = ros::service::call("square_number", req, res);

		if (success == true)
			std::cout << "The square of " << num << " is " << res.numberSquared << ".\n" << std::endl;
		else
			std::cout << "The service call was not successful.\n" << std::endl;
	}


	return 0;
}
