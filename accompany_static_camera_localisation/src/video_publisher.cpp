#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "video_publisher");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("/gscam/image_raw", 1);

	ros::Rate loop_rate(5);
	while (n.ok())
	{
		cv::WImageBuffer3_b im( cvLoadImage("test_frame/frame0000.jpg", CV_LOAD_IMAGE_COLOR) ); // TODO: To be removed
		sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(im.Ipl(), "bgr8");
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
