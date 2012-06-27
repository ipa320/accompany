#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <accompany_example_service/Message.h>

class Example_service_server
{
public:
	Example_service_server(ros::NodeHandle nh);
	void init();
	bool square_number(accompany_example_service::Message::Request &req, accompany_example_service::Message::Response &res);

protected:
	ros::NodeHandle node_;
	ros::ServiceServer service_server_square_; ///< Service server which accepts requests for squaring a number
};
