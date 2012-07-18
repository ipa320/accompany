#include "accompany_example_action/example_action_server.h"

ExampleActionServer::ExampleActionServer(ros::NodeHandle nh)
: action_server_square_(nh, "square_number_action", boost::bind(&ExampleActionServer::square_number, this, _1), false)	// this initializes the action server; important: always set the last parameter to false
{
	node_ = nh;
}


void ExampleActionServer::init()
{
	// by starting the action server, your action gets advertised to other software modules

	action_server_square_.start();
}


void ExampleActionServer::square_number(const accompany_example_action::MessageGoalConstPtr& goal)
{
	// this callback function is executed each time a request (= goal message) comes in for this service server
	// here we just read the number from the goal message, square it and put the result into the result message

	ROS_INFO("Action Server: Received a request with number %f.", goal->number);

	// this command sends a feedback message to the caller, here we transfer that the task is completed 25%
	accompany_example_action::MessageFeedback feedback;
	feedback.percentageDone = 25;
	action_server_square_.publishFeedback(feedback);

	// slow down the response to show the effect of feedback messages
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();

	// do the computation
	double num = goal->number;
	double num2 = num * num;

	// send another feedback message
	feedback.percentageDone = 75;
	action_server_square_.publishFeedback(feedback);

	// slow down the response to show the effect of feedback messages
	loop_rate.sleep();

	accompany_example_action::MessageResult res;
	res.numberSquared = num2;

	// this sends the response back to the caller
	action_server_square_.setSucceeded(res);
}
