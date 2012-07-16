
#include <ros/ros.h>
#include <HumanTracker/HumanLocations.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "Localisation");

  // read files

  // create publisher
  ros::NodeHandle n;
  ros::Publisher humanLocationsPub = n.advertise<HumanTracker::HumanLocations>("/humanLocations", 10);

  int test=0;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    // read image from camera

    // process image

    // publish human locations
    HumanTracker::HumanLocations humanLocations;
    geometry_msgs::Vector3 v;
    v.x=1+(test*0.1);
    v.y=2;
    v.z=0;
    humanLocations.locations.push_back(v);
    v.x=3;
    v.y=3+(test*0.2);
    v.z=0;
    humanLocations.locations.push_back(v);
    test=(test+1)%100;
    humanLocationsPub.publish(humanLocations);

    // publish samples particles
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
