
#include <ros/ros.h>
#include <HumanTracker/HumanLocations.h>
#include <StaticCameraLocalisation/HumanLocationsParticle.h>
#include <StaticCameraLocalisation/HumanLocationsParticles.h>

#include <ctime>
#include <cstdlib>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "Localisation");

  srand(time(0));// initialize random number generator

  // read files

  // create publishers and subscribers
  ros::NodeHandle n;
  ros::Publisher humanLocationsPub=n.advertise<HumanTracker::HumanLocations>("/humanLocations",10);
  ros::Publisher humanLocationsParticlesPub=n.advertise<StaticCameraLocalisation::HumanLocationsParticles>("/humanLocationsParticles",10);

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

    // publish human locations particles
    StaticCameraLocalisation::HumanLocationsParticles humanLocationsParticles;
    int numberOfParticles=10;
    for (int i=0;i<numberOfParticles;i++)
    {
      StaticCameraLocalisation::HumanLocationsParticle humanLocationsParticle;
      int numberOfLocations=(rand()%humanLocations.locations.size())+1;
      for (int l=0;l<numberOfLocations;l++)
      {
        int randomLoc=(rand()%humanLocations.locations.size());// random location index
        geometry_msgs::Vector3 v=humanLocations.locations[randomLoc];// random location vector
        humanLocationsParticle.locations.push_back(v);// add location to particle
      }
      humanLocationsParticle.weight=rand()/((double)RAND_MAX);// set random weight
      humanLocationsParticles.particles.push_back(humanLocationsParticle);
    }
    humanLocationsParticlesPub.publish(humanLocationsParticles);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
