
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <accompany_uva_utils/uva_utils.h>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

int main(int argc, char **argv)
{
  string parent;
  string frameName;
  double a;
  double x,y,z;

  // handling arguments
  options_description optionsDescription(
      "Create a coordinate frame relative to a parent\n"
      "Allowed options");
  optionsDescription.add_options()
    ("frame-name,f", value<string>(&frameName)->required(),"name of the new frame")
    ("parent,p", value<string>(&parent)->default_value("/map"),"name of parent frame")
    ("angle,a", value<double>(&a)->default_value(0.0),"angle in xy plane")
    ("xpos,x", value<double>(&x)->default_value(0.0),"x position")
    ("ypos,y", value<double>(&y)->default_value(0.0),"y position")
    ("zpos,z", value<double>(&z)->default_value(0.0),"z position");
  
  variables_map variablesMap;

  try
  {
    store(parse_command_line(argc, argv, optionsDescription),
        variablesMap);
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


  ros::init(argc, argv, "create_frame");
  
  string filename="frame.dat";
  cout<<"create some frame and write to file '"<<filename<<"'"<<endl;
  geometry_msgs::TransformStamped transformStamped;

  tf::Transform transform=tf::Transform(btMatrix3x3( cos(a),sin(a),0, // rotation matrix
                                                    -sin(a),cos(a),0,
                                                    0,0,1),  
                                        btVector3(x,y,z)); // translation vector
  tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                             ros::Time(0),  // time, not used here
                                                             parent,        // parent coordinate frame
                                                             frameName); // child coordinate frame
  tf::transformStampedTFToMsg(stampedTransform,transformStamped);
  save_msg(transformStamped,filename); // write to file
 

  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transformStamped2;
  load_msg(transformStamped2,filename);
  cout<<transformStamped2;
  
}
