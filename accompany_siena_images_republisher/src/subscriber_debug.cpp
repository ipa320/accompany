#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	sensor_msgs::CvBridge bridge;
	printf("+\n");
	try
	{
		cvShowImage("Accompany Debug", bridge.imgMsgToCv(msg,"bgr8"));
	}
	catch(sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("cannot convert");
	}
}	

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Accompany_Image_debug");
	ros::NodeHandle nh;
	cvNamedWindow("Accompany Debug");
	cvStartWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("accompany/GUIimage", 1, image_callback);
	ros::spin();
	cvDestroyWindow("Accompany Debug");
}
