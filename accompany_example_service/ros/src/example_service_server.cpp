#include "accompany_example_service/example_service_server.h"

ExampleServiceServer::ExampleServiceServer(ros::NodeHandle nh)
{
	node_ = nh;
}


void ExampleServiceServer::init()
{
	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/

	service_server_square_ = node_.advertiseService("square_number", &ExampleServiceServer::square_number, this);
}


bool ExampleServiceServer::square_number(accompany_example_service::Message::Request &req, accompany_example_service::Message::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	ROS_INFO("Service Server: Received a request with number %f.", req.number);

	double num = req.number;
	double num2 = num * num;
	res.numberSquared = num2;

	return true;
}
