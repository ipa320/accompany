
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_static_camera_localisation/HumanLocationsParticle.h>
#include <accompany_static_camera_localisation/HumanLocationsParticles.h>

#include <ctime>
#include <cstdlib>
#include <iostream>
using namespace std;
#include <boost/program_options.hpp>
namespace po = boost::program_options;


int main(int argc,char **argv)
{
  // options
  bool particles=false;
  int nrParticles;
  // boost parameters parsing
  po::options_description optionsDescription("Allowed options");
  optionsDescription.add_options()
    ("help", "produce help message")
    ("particles,p","publish particles and do not publish human locations directly")
    ("nrparticles,n", po::value<int>(&nrParticles)->default_value(10),"number of particles to sample (implies -p)")
    ;
  po::variables_map variablesMap;
  po::store(po::parse_command_line(argc, argv, optionsDescription), variablesMap);
  po::notify(variablesMap);
  
  if (variablesMap.count("help"))
  {
    cout<<optionsDescription<<endl;
    return 1;
  }
  if (variablesMap.count("particles") || variablesMap.count("nrparticles"))
    particles=true;

  // init ros
  ros::init(argc, argv, "CameraLocalisation");

  // read files

  // create publishers and subscribers
  ros::NodeHandle n;
  ros::Publisher humanLocationsPub=n.advertise<accompany_human_tracker::HumanLocations>("/humanLocations",10);
  ros::Publisher humanLocationsParticlesPub=n.advertise<accompany_static_camera_localisation::HumanLocationsParticles>("/humanLocationsParticles",10);

  // generate dummy data
  int max=100;
  int count=0;
  int direction=1;

  srand(time(0));// initialize random number generator

  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    // read image from camera

    // process image

    // publish human locations
    accompany_human_tracker::HumanLocations humanLocations;
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
    if (!particles)
    humanLocationsPub.publish(humanLocations);

    // publish human locations particles
    if (particles)
    {
      accompany_static_camera_localisation::HumanLocationsParticles humanLocationsParticles;
      for (int i=0;i<nrParticles;i++)
      {
        accompany_static_camera_localisation::HumanLocationsParticle humanLocationsParticle;
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
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
