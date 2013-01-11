#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <boost/program_options.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace boost::program_options;

string source_frame, target_frame;
double px, py, pz;

void transformPoint(const tf::TransformListener& listener)
{

  geometry_msgs::PointStamped p;
  p.header.frame_id = source_frame;
  p.header.stamp = ros::Time();

  // point coordinates
  p.point.x = px;
  p.point.y = py;
  p.point.z = pz;

  try
    {
      geometry_msgs::PointStamped tfp;
      listener.transformPoint(target_frame, p, tfp);

      ROS_INFO("transform point from %s(%.2f, %.2f. %.2f) to %s(%.2f, %.2f, %.2f) at time %.2f",
               p.header.frame_id.c_str(), p.point.x, p.point.y, p.point.z,
               target_frame.c_str(), tfp.point.x, tfp.point.y, tfp.point.z, tfp.header.stamp.toSec());
    }
  catch(tf::TransformException& ex)
    {
      ROS_WARN("Could not find a connection between %s and %s: %s", p.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    }
}

int main(int argc, char** argv){

  // handling arguments
  options_description optionsDescription
    ( "Transform tf points\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("source_frame,s", value<string>(&source_frame)->required(), "source frame where the points locates")
    ("target_frame,t", value<string>(&target_frame)->required(), "target frame where the points to be transformed to")
    ("px,x", value<double>(&px)->required(), "coordinate x")
    ("py,y", value<double>(&py)->required(), "cooridnate y")
    ("pz,z", value<double>(&pz)->required(), "coordinate z");

  variables_map variablesMap;
  try
    {
      store(parse_command_line(argc, argv, optionsDescription),variablesMap);
      if (variablesMap.count("help")) {std::cout<<optionsDescription<<std::endl; return 0;}
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

  ros::init(argc, argv, "transform_points");

  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10)); // TODO what is duration here?

  ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&transformPoint, boost::ref(listener))); // 100Hz

  ros::spin();

}
