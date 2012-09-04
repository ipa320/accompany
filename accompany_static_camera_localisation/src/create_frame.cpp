
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <accompany_uva_utils/uva_utils.h>

#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_frame");
  
  char *filename="frame.dat";
  cout<<"create some frame and write to file '"<<filename<<"'"<<endl;
  geometry_msgs::TransformStamped transformStamped;

  double a=0;
  tf::Transform transform=tf::Transform(btMatrix3x3( cos(a),sin(a),0, // rotation matrix
                                                    -sin(a),cos(a),0,
                                                    0,0,1),  
                                        btVector3(0,0,0)); // translation vector
  tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                             ros::Time(0),  // time, not used here
                                                             "/map",        // parent coordinate frame
                                                             "/overhead1"); // child coordinate frame
  tf::transformStampedTFToMsg(stampedTransform,transformStamped);
  save_msg(transformStamped,filename); // write to file
 

  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transformStamped2;
  load_msg(transformStamped2,filename);
  cout<<transformStamped2;
  
}
