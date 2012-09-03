
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
  tf::Transform transform=tf::Transform(btMatrix3x3(1,0,0, // rotation matrix
                                                    0,1,0,
                                                    0,0,1),  
                                        btVector3(0,0,0)); // translation vector
  tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                             ros::Time(0),  // time, not used 
                                                             "/map",        // parent coordinate frame
                                                             "/overhead1"); // child coordinate frame
  tf::transformStampedTFToMsg(stampedTransform,transformStamped);
  save_msg(transformStamped,filename); // write to file
 

  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transformStamped2;
  load_msg(transformStamped2,filename);
  cout<<transformStamped2;
  
}
