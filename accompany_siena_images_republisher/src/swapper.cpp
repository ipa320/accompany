#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Accompany_Image_Republisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub= it.advertise("accompany/GUIimage",1);
	cv::WImageBuffer3_b image( cvLoadImage("/home/patrizia/frame0000.jpg", CV_LOAD_IMAGE_COLOR));

	sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(),"bgr8");

	ros::Rate loop_rate(0.2);
	while (nh.ok())
	{
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
