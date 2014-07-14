#include <cstdio>
#include "tf/transform_broadcaster.h"
#include "opencv2/opencv.hpp"
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace std;

class TransformSender
{
  public:
    ros::NodeHandle node_;

    //constructor
    TransformSender(cv::Mat tvec, cv::Mat rmat, ros::Time time, std::string& frame_id, std::string& child_frame_id)
    { 

      cout << endl << "tf: " << frame_id << " -> " << child_frame_id << endl;
      cout << "rotation matrix:" << endl << rmat << endl;
      tf::Transform transform = tf::Transform(
					      tf::Matrix3x3(
							    rmat.at<double>(0,0),rmat.at<double>(0,1),rmat.at<double>(0,2),// rotation matrix
							    rmat.at<double>(1,0),rmat.at<double>(1,1),rmat.at<double>(1,2),
							    rmat.at<double>(2,0),rmat.at<double>(2,1),rmat.at<double>(2,2)), 
					      tf::Vector3(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0))// translation vector
          );
      std::cout << "translation vector:" << endl << tvec << endl;
      tf::Matrix3x3 ma= transform.getBasis();

      transform_ = tf::StampedTransform(transform, time, frame_id, child_frame_id );
    };

    //Clean up ros connections
    ~TransformSender() { }

    //A pointer to the rosTFServer class
    tf::TransformBroadcaster broadcaster;

    // A function to call to send data periodically
    void send (ros::Time time) 
    {
      transform_.stamp_ = time;
      broadcaster.sendTransform(transform_);
    };

  private:
    tf::StampedTransform transform_;

};

int main(int argc, char ** argv)
{
  std::string parent_frame_id, child_frame_id, filename;
  double pub_period;

  // arguments
  po::options_description optionsDescription(
      " create tf between the camera and the world frame using extrinsic parameters\n"
      " example: tf_camera_transform   "
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("parent_frame_id,p", po::value<string>(&parent_frame_id)->required(),"parent frame\n")
    ("child_frame_id,c", po::value<string>(&child_frame_id)->required(),"child frame\n")
    ("extrinsic_xml,e", po::value<string>(&filename)->required(),"xml file with extrinsic calibration parameters, camera_extrinsic.xml usually\n")
    ("frequency,f", po::value<double>(&pub_period)->default_value(100), "tf publish frequency\n")
    ("reverse,r", "switch on when World is a child of Camera)\n");

  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    po::notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }

  // load extrinsic parameters from file
  cv::Mat tvec, rvec, rmat;
  cv::FileStorage fs(filename,cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_FATAL("cannot open file %s", filename.c_str());
  }
  fs["tvec"] >> tvec; tvec = tvec / 1000; // scale translation to meters
  fs["rvec"] >> rvec;
  fs.release();
  cv::Rodrigues(rvec,rmat); // rotation vector to rotation matrix

  if (variablesMap.count("reverse") == 0) // default: World->Camera 
  {
    rmat = rmat.inv();
    tvec = (-1) * rmat * tvec;
  }

  //Initialize ROS
  ros::init(argc, argv,"tf_camera_bridge", ros::init_options::AnonymousName);
  ros::Duration sleeper(pub_period/1000.0);

  if (strcmp(parent_frame_id.c_str(), child_frame_id.c_str()) == 0)
    ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", parent_frame_id.c_str(), child_frame_id.c_str());

  TransformSender tf_sender(tvec, rmat, ros::Time() + sleeper /* time travel!*/, parent_frame_id, child_frame_id);

  while(tf_sender.node_.ok())
  {
    tf_sender.send(ros::Time::now() + sleeper);
    ROS_DEBUG("Sending transform from %s with parent %s\n", parent_frame_id.c_str(), child_frame_id.c_str());
    sleeper.sleep();
  }

  return 0;
};
