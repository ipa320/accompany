
#include <ros/ros.h>
#include <HumanTracker/HumanLocations.h>
#include <HumanTracker/TrackedHumans.h>

#include <MyTracker.h>

#include <iostream>
using namespace std;

// globals
ros::Publisher trackedHumansPub;
MyTracker myTracker;

void humanLocationsReceived(const HumanTracker::HumanLocations::ConstPtr& humanLocations)
{
  for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    cout<<"humanLocations["<<i<<"].x="<<humanLocations->locations[i].x<<endl;
    cout<<"humanLocations["<<i<<"].y="<<humanLocations->locations[i].y<<endl;
    cout<<"humanLocations["<<i<<"].z="<<humanLocations->locations[i].z<<endl;
  }
  myTracker.trackHumans(humanLocations);
  trackedHumansPub.publish(myTracker.getTrackedHumans());
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "Tracker");

  // create publisher and subscribers
  ros::NodeHandle n;
  trackedHumansPub=n.advertise<HumanTracker::TrackedHumans>("/trackedHumans",10);
  ros::Subscriber humanLocationsSub=n.subscribe<HumanTracker::HumanLocations>("/humanLocations",10,humanLocationsReceived);
  
  // read human indentities

  // do tracking

  // publish tracked humans

  ros::spin(); // wait for shutdown

  return 0;
}
