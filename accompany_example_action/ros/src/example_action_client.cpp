#include "accompany_example_action/example_action_client.h"

ExampleActionClient::ExampleActionClient(ros::NodeHandle nh)
: action_client_square_("square_number_action", true)  // true -> don't need ros::spin()
{
	node_ = nh;
}


bool ExampleActionClient::init()
{
	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for action server to become available..." << std::endl;
	return action_client_square_.waitForServer(ros::Duration(5.0));
}


void ExampleActionClient::run()
{
	// prepare the goal message
	accompany_example_action::MessageGoal goal;

	while(1)
	{
		double num;
		std::cout << "Enter a number that shall be squared (enter 0 to quit): \n";
		std::cin >> num;
		if (num == 0.0)
			return;
		goal.number = num;

		// this calls the action server to process our goal message and send result message which will cause the execution of the callback function
		// this call is not blocking, i.e. this program can proceed immediately after the action call
		action_client_square_.sendGoal(goal, boost::bind(&ExampleActionClient::doneCb, this, _1, _2), boost::bind(&ExampleActionClient::activeCb, this), boost::bind(&ExampleActionClient::feedbackCb, this, _1));
	}
}

// Called once when the goal completes
void ExampleActionClient::doneCb(const actionlib::SimpleClientGoalState& state, const accompany_example_action::MessageResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %f", result->numberSquared);
  std::cout << "Computation finished. You can enter the next number." << std::endl;
}

// Called once when the goal becomes active
void ExampleActionClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void ExampleActionClient::feedbackCb(const accompany_example_action::MessageFeedbackConstPtr& feedback)
{
  ROS_INFO("Computation accomplished by %f percent.", feedback->percentageDone);
}
