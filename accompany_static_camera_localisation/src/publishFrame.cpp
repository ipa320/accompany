
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <iostream>
using namespace std;

#include <accompany_uva_utils/uva_utils.h>

int main(int argc,char** argv)
{
  string frame_file;
  double hertz;

  // handling arguments
  po::options_description optionsDescription("Publish frame data");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("frame_file,f", po::value<string>(&frame_file)->required(),"filename of the stored coordinate frame (in TransformStamped binary format)")
    ("hertz,h", po::value<double>(&hertz)->default_value(10),"frequency of publishing");
  
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

  geometry_msgs::TransformStamped frame;
  // load coordinate frame of camera
  if (!load_msg(frame,frame_file))
  {
    std::cerr<<"Failed to load the coordinate frame from file '"<<frame_file<<"'. Use program 'create_frame' to create the file."<<endl;
    exit(1);
  }
  
  ros::init(argc, argv, "publishFrame");
  tf::TransformBroadcaster transformBroadcaster;
  ros::Rate r(hertz);
  while (ros::ok())
  {
    frame.header.stamp=ros::Time::now();
    transformBroadcaster.sendTransform(frame);
    ros::spinOnce();
    r.sleep();
  }

}
