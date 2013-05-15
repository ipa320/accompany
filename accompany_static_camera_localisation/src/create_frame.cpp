
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
  string parentName;
  string frameName;
  string filename;
  double a;
  double x,y,z;

  // handling arguments
  options_description optionsDescription(
      "Create a coordinate frame relative to a parent and save it to disk\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("filename,f", value<string>(&filename)->default_value("frame.dat"),"filename of the coordiante name file")
    ("frame-name,n", value<string>(&frameName)->default_value("/myFrame"),"name of the new frame")
    ("parent-name,p", value<string>(&parentName)->default_value("/map"),"name of parent frame")
    ("angle,a", value<double>(&a)->default_value(0.0),"angle in xy plane in degrees")
    ("xpos,x", value<double>(&x)->default_value(0.0),"x position")
    ("ypos,y", value<double>(&y)->default_value(0.0),"y position")
    ("zpos,z", value<double>(&z)->default_value(0.0),"z position")
    ("view-only,v","don't create a new coordinate frame, only print current file to screen")
    ("rename,r","rename the child and parent of existing frame when new names are given, implies '-v'");

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


  ros::init(argc, argv, "create_frame");

  if ((!variablesMap.count("view-only")) && (!variablesMap.count("rename")) )
  {
    // create frame
    cout<<"create some frame and write to file '"<<filename<<"'"<<endl;
    geometry_msgs::TransformStamped transformStamped;

    tf::Transform transform=tf::Transform(btMatrix3x3( cos(a*M_PI/180),sin(a*M_PI/180),0, // rotation matrix
                                                      -sin(a*M_PI/180),cos(a*M_PI/180),0,
                                                                     0,              0,1),
                                          btVector3(x,y,z)); // translation vector
    tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                               ros::Time(0),  // time, not used here
                                                               parentName,        // parent coordinate frame
                                                               frameName); // child coordinate frame
    tf::transformStampedTFToMsg(stampedTransform,transformStamped);
    save_msg(transformStamped,filename); // write to file
  }
  if (variablesMap.count("rename"))
  {
    // load and rename frames
    cout<<"load and rename frame names"<<endl;
    geometry_msgs::TransformStamped transformStamped;
    load_msg(transformStamped,filename);
    bool rename=false;
    if (variablesMap.count("frame-name"))
    {
      cout<<"renaming child-frame-name: '"<<frameName<<"'"<<endl;
      transformStamped.child_frame_id=frameName;
      rename=true;
    }
    if (variablesMap.count("parent-name"))
    {
      cout<<"renaming parent-name: '"<<parentName<<"'"<<endl;
      transformStamped.header.frame_id=parentName;
      rename=true;
    }
    if (rename)
      save_msg(transformStamped,filename); // write to file
  }

  // load frame and print
  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transformStamped;
  load_msg(transformStamped,filename);
  cout<<transformStamped;
 
  geometry_msgs::Quaternion rot=transformStamped.transform.rotation;
  btMatrix3x3 mat(btQuaternion(rot.x,rot.y,rot.z,rot.w));
  cout<<"conversion to 3x3 rotation matrix:"<<endl;
  for (int i=0;i<3;i++)    
    cout<<"|"<<setw(12)<<mat[i].x()<<" "<<setw(12)<<mat[i].y()<<" "<<setw(12)<<mat[i].z()<<"|"<<endl;
 
 
}
