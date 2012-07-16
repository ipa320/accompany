
#include <ros/ros.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "Localisation");

  // read files

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    // read image from camera

    // process image

    // publish human locations
    
    // publish samples particles
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
