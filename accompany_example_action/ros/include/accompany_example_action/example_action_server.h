#include "ros/ros.h"

// actions
#include <actionlib/server/simple_action_server.h>
#include <accompany_example_action/MessageAction.h> // here you have to include the header file with exactly the same name as your message in the /action folder (the Message.h is automatically generated from your Message.action file during compilation)

// this typedef just establishes the abbreviation SquareActionServer for the long data type
typedef actionlib::SimpleActionServer<accompany_example_action::MessageAction> SquareActionServer;

class ExampleActionServer
{
public:
	ExampleActionServer(ros::NodeHandle nh);
	void init();
	void square_number(const accompany_example_action::MessageGoalConstPtr& goal);

protected:
	ros::NodeHandle node_;
	SquareActionServer action_server_square_; ///< Action server which accepts requests for squaring a number
};
