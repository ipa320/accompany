

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

int width,height;
cv::Mat resize;
image_transport::Publisher *imagePub;

cv_bridge::CvImagePtr cv_ptr;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvShare(msg,"bgr8");
    cv::resize(cv_ptr->image,resize, cv::Size(width,height));
    cv_bridge::CvImage resizeRos;
    resizeRos.encoding = "bgr8";
    resizeRos.image = resize;
    imagePub->publish(resizeRos.toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
} 

int main(int argc,char **argv)
{
  string topic_in,topic_out;
  
  // handling arguments
  options_description optionsDescription("Reads image from input topic, resize it and publishes it to output topic");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("topic_in,i", value<string>(&topic_in)->required(),"name of input topic")
    ("topic_out,o", value<string>(&topic_out)->required(),"name of input topic")
    ("with", value<int>(&width)->required(),"the width of the output")
    ("height", value<int>(&height)->required(),"the height of the output");
    
  variables_map variablesMap;
  try
  {
    store(parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }
  

  ros::init(argc, argv, "resize_image");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(topic_out, 1);
  imagePub=&pub;
  image_transport::Subscriber sub = it.subscribe(topic_in, 1, &callback);
  ros::spin();

  return 0;
}
