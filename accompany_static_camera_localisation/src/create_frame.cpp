
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <accompany_uva_utils/uva_utils.h>

#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_frame");
  
  char *filename="frame.dat";
  cout<<"create some frame and write to file '"<<filename<<"'"<<endl;
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id="/map";
  transform.child_frame_id="/overhead1";
  transform.transform.translation.x=0;
  transform.transform.translation.y=0;
  transform.transform.translation.z=0;
  transform.transform.rotation.x=0;
  transform.transform.rotation.y=0;
  transform.transform.rotation.z=0;
  transform.transform.rotation.w=1;
  save_msg(transform,filename); // write to file

  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transform2;
  load_msg(transform2,filename);
  cout<<transform2;
  
}
