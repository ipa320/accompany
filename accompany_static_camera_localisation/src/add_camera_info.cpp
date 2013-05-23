

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

variables_map variablesMap;
string frame_id;

sensor_msgs::CameraInfo camera_info;
image_transport::CameraPublisher *imagePub;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::Image image=*msg;

  camera_info.header.stamp = ros::Time::now();
  image.header.stamp = camera_info.header.stamp;
  image.header.frame_id = camera_info.header.frame_id;

  imagePub->publish(image,camera_info);
} 

void xml2yaml(std::string calib_xml, std::string& calib_yaml)
{
  int image_width, image_height;
  std::string camera_name, distortion_model;
  cv::Mat camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix;

  boost::filesystem::path p(calib_xml);
  boost::filesystem::path dir = p.parent_path();
  calib_yaml = dir.string() + "/camera_intrinsic.yaml";

  cv::FileStorage fs1(calib_xml, cv::FileStorage::READ);
  cv::FileStorage fs2(calib_yaml,cv::FileStorage::WRITE);
  if (!fs1.isOpened())
  {
    ROS_ERROR("cannot open %s", calib_xml.c_str());
  }

  ROS_INFO("convert %s to %s", calib_xml.c_str(), calib_yaml.c_str());
  fs1["image_width"] >> image_width;
  fs2 << "image_width" << image_width;
  fs1["image_height"] >> image_height;
  fs2 << "image_height" << image_height;
  fs2 << "camera_name" << "camera";
  fs1["camera_matrix"] >> camera_matrix;
  fs2 << "camera_matrix" << camera_matrix;

  fs2 << "distortion_model" << "plumb_bob";
  fs1["distortion_coefficients"] >> distortion_coefficients;
  fs2 << "distortion_coefficients" << distortion_coefficients;

  cv::hconcat(camera_matrix,cv::Mat::zeros(3,1,CV_64F),projection_matrix);

  fs2 << "rectification_matrix" << cv::Mat::eye(3,3,CV_32F);
  fs2 << "projection_matrix" << projection_matrix;
  
  fs1.release();
  fs2.release();

}

int main(int argc,char **argv)
{
  string topic_in,topic_out;
  string calib_xml;

  // handling arguments
  options_description optionsDescription("Resize add frame and publish camera info on an existing image topic");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("topic_in,i", value<string>(&topic_in)->required(),"name of input image topic")
    ("topic_out,o", value<string>(&topic_out)->required(),"name of output image topic")
    ("frame_id,f", value<string>(&frame_id)->default_value("optical_frame"),"the frame name of the stream")
    ("intrinsic_xml,n", value<std::string>(&calib_xml)->default_value("camera_intrinsic.xml"),"load intrinsic parameters, use image_proc for image undistortion\n");

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
  
  // read intrinsic calibration data
  string calib_yaml;
  xml2yaml(calib_xml,calib_yaml);
  std::string camera_name;
  if (camera_calibration_parsers::readCalibrationYml(calib_yaml, camera_name, camera_info)) 
  {
    ROS_INFO("Successfully read camera calibration. Return camera calibrator if it is incorrect.");
    camera_info.header.frame_id = frame_id; 
    if (variablesMap.count("frame_id") == 0)
      ROS_WARN("No frame_id specified, use default: %s\n Use option -f to specify which frame to associate", frame_id.c_str());
    else
      ROS_INFO("Use the specified frame_id: %s", frame_id.c_str());
  }
  else 
  {
    ROS_WARN("No camera_parameters.txt file found.  Use default file if no other is available.");
    ROS_WARN("Use -i option to load the intrinsic parameters");
  }
  // set frame name
  camera_info.header.frame_id = frame_id; 

  ros::init(argc, argv, "resize_image");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera(topic_out, 1);
  imagePub=&pub;
  image_transport::Subscriber sub = it.subscribe(topic_in, 1, &callback);
  ros::spin();

  return 0;
}
