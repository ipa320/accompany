
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
using namespace std;

//globals
tf::TransformListener transformListener; // reads corrected odometry

void laserScanReceived(const sensor_msgs::LaserScan& scan_msg)
{
  //received laser scan
  cout<<"LaserScan stamp: "<<scan_msg.header.stamp<<endl;

  try // read robot location relative to map
  {
    tf::StampedTransform transform;
    transformListener.lookupTransform("/map","/base_link",ros::Time(0),transform);
    double phi=tf::getYaw(transform.getRotation());
    cout<<"x: "<<transform.getOrigin().x()<<endl;
    cout<<"y:"<<transform.getOrigin().y()<<endl;
    cout<<"phi:"<<phi<<endl;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "BuildBackgroundModel");

  // create publisher and subscribers
  ros::NodeHandle n;
  ros::Subscriber laserScanSub= n.subscribe("/scan", 10,laserScanReceived);

  ros::spin(); // wait for shutdown

  return 0;

}
