
#include <ros/ros.h>
#include <HumanTracker/HumanLocations.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "Localisation");

  // read files

  // create publishers and subscribers
  ros::NodeHandle n;
  ros::Publisher humanLocationsPub=n.advertise<HumanTracker::HumanLocations>("/humanLocations",10);

  // generate dummy data
  int max=100;
  int count=0;
  int direction=1;

  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    // read image from camera

    // process image

    // publish human locations
    HumanTracker::HumanLocations humanLocations;
    geometry_msgs::Vector3 v;
    v.x=10+((direction<0)*max+count*direction)*0.1;
    v.y=1;
    v.z=0;
    humanLocations.locations.push_back(v);
    v.x=3;
    v.y=10+((direction<0)*max+count*direction)*0.2;
    v.z=0;
    humanLocations.locations.push_back(v);
    v.x=5+count*0.15;
    v.y=5+count*0.15;
    v.z=0;
    humanLocations.locations.push_back(v);
    if (++count>=max)
    {
      count=0;
      direction*=-1;
    }
    humanLocationsPub.publish(humanLocations);

    // publish samples particles
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
