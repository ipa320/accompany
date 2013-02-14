
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <accompany_uva_msg/HumanLocationsParticles.h>
#include <accompany_uva_msg/HumanLocations.h>

#include <ctime>
#include <cstdlib>
#include <iostream>
using namespace std;

//globals
tf::TransformListener *transformListenerPtr; // reads corrected odometry
ros::Publisher humanLocationsPub;

void laserScanReceived(const sensor_msgs::LaserScan& scan_msg)
{
  //received laser scan
  cout<<"LaserScan stamp: "<<scan_msg.header.stamp<<endl;

  try // read robot location relative to map
  {
    tf::StampedTransform transform;
    transformListenerPtr->lookupTransform("/map","/base_link",ros::Time(0),transform);
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

void humanLocationsParticlesReceived(const accompany_uva_msg::HumanLocationsParticles& particles)
{
  // select best particle
  accompany_uva_msg::HumanLocationsParticle bestParticle;
  bestParticle.weight=0;
  for (int i=0;i<particles.particles.size();i++)
  {
    if (particles.particles[i].weight>bestParticle.weight)
    {
      bestParticle=particles.particles[i];
    }
  }
  // publish locations of best particle
  accompany_uva_msg::HumanLocations humanLocations;
  humanLocations.locations=bestParticle.locations;
  humanLocationsPub.publish(humanLocations);
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "LaserLocalisation");

  // create publisher and subscribers
  ros::NodeHandle n;
  tf::TransformListener transformListener;
  transformListenerPtr=&transformListener;
  ros::Subscriber laserScanSub= n.subscribe("/scan", 10,laserScanReceived);
  ros::Subscriber humanLocationsParticlesSub= n.subscribe("/humanLocationsParticles", 10,humanLocationsParticlesReceived);
  humanLocationsPub=n.advertise<accompany_uva_msg::HumanLocations>("/humanLocations",10);

  ros::spin(); // wait for shutdown

  return 0;
}
