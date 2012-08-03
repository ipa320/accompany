#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <string.h>
#include <iostream>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
	double scale;
	string input_video;

	// handling arguments
	po::options_description optionsDescription("Allowed options\n");
	optionsDescription.add_options()
		("scale,s", po::value<double>(&scale)->required(),"resize scale\n")
		("input_video,i", po::value<string>(&input_video)->required(),"input video\n");
	try
	{
		po::variables_map variablesMap;
		po::store(po::parse_command_line(argc, argv, optionsDescription), variablesMap);
		po::notify(variablesMap);
	}
	catch( const std::exception& e)
	{
		std::cout << "--------------------" << std::endl;
		std::cout <<  optionsDescription << std::endl;
		std::cout << "--------------------" << std::endl;
		std::cerr << "- "<<e.what() << std::endl;
		return 1;
	}

	ros::init(argc, argv, "video_publisher");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("/gscam/image_raw", 1);

	cv::VideoCapture cap;
	cap.open(input_video);
	if(!cap.isOpened())  // check if we succeeded
	{
		ROS_ERROR("video capture failed");
	}

	cv::Mat frame,sframe;
	ros::Rate loop_rate(5);
	while (n.ok())
	{
		cap >> frame;
		cv::resize(frame,sframe,cv::Size((int) (frame.cols*scale),(int) (frame.rows*scale)));
		IplImage ipl_im = sframe;
		IplImage* ipl_ptr = &ipl_im;
		sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(ipl_ptr, "bgr8");
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
