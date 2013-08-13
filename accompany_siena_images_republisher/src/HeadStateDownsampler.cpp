#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

#define DEFAULT_OUTPUT_TOPIC "head_controller"
#define DEFAULT_INPUT_TOPIC "/stereo/left/image_color"

//republish rate constants
#define RATE 100        // not usefull to have too much

std::string head_output_topic;
std::string head_input_topic;

int sampling_rate;
int n;
double head_position;

ros::Publisher pub;

void head_callback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
	pr2_controllers_msgs::JointTrajectoryControllerState traj= *msg;
	double a = traj.actual.positions[0];
	//printf("new head position: %f\n",a);
	head_position=a;

	if (n%sampling_rate==0)
	{
		pub.publish(traj);
	}
}

int main(int argc, char** argv)
{
	//setting up default values
	head_input_topic=DEFAULT_INPUT_TOPIC;
	head_output_topic=DEFAULT_OUTPUT_TOPIC;
	sampling_rate=RATE;
        //reading the configuration
	std::string path=ros::package::getPath("accompany_siena_images_republisher")+"/config/config.txt";
	//std::cout << path << "\n";
	
	std::ifstream infile(path.c_str());

	std::string param,value;
	while(infile >> param >> value)
	{
		//std::cout << "param: " << param << ", value: " << value <<"\n";
		if (param=="head_input_topic")
		{
			head_input_topic=value;
		}
		if (param=="head_downsampled_control_topic")
		{
			head_output_topic=value;
		}
		if (param=="head_sampling_rate")
		{
			sscanf(value.c_str(), "%d", &sampling_rate);
		}
	}
	std::cout <<"Input topic: " <<head_input_topic<<"\n";
	std::cout <<"Output topic: " <<head_output_topic<<"\n";
	std::cout <<"Sampling rate: "<<sampling_rate<<"\n";
	
	ros::init(argc,argv,"Head_state_downsampler");
	ros::NodeHandle nh;

	//reset the counter;
	n=sampling_rate;

	//creaTING THE PUBLISHER
	pub = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(head_output_topic,100);
	
	//Subscribing head position:
	ros::Subscriber head_sub=nh.subscribe(head_input_topic,1,head_callback);
	
	std::cout <<"\n\n";
	std::cout << "Remapping topic \"" <<head_input_topic << "\" on \"" << head_output_topic <<"\"\n";
	std::cout << "Cutting head state frequency with "<<sampling_rate <<" ratio.\n";
	std::cout << "Head state downsampler started...\n"	;

	ros::spin();

        
}
