#include "ros/ros.h"

// actions
#include <actionlib/client/simple_action_client.h>
#include <accompany_example_action/MessageAction.h> // here you have to include the header file with exactly the same name as your message in the /action folder (the Message.h is automatically generated from your Message.action file during compilation)

// this typedef just establishes the abbreviation SquareActionServer for the long data type
typedef actionlib::SimpleActionClient<accompany_example_action::MessageAction> SquareActionClient;

class ExampleActionClient
{
public:
	ExampleActionClient(ros::NodeHandle nh);
	bool init();
	void run();

protected:
	void doneCb(const actionlib::SimpleClientGoalState& state, const accompany_example_action::MessageResultConstPtr& result);
	void activeCb();
	void feedbackCb(const accompany_example_action::MessageFeedbackConstPtr& feedback);

	ros::NodeHandle node_;
	SquareActionClient action_client_square_; ///< Action server which accepts requests for squaring a number
};
