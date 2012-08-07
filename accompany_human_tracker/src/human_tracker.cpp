
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHumans.h>

#include <MyTracker.h>
#include <TimTracker/Tracker.h>

#include <iostream>
using namespace std;

// globals
ros::Publisher trackedHumansPub;
MyTracker myTracker;

Tracker tracker;
ros::Time prevNow;

void humanLocationsReceived(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations)
{
  /*for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    cout<<"humanLocations["<<i<<"].x="<<humanLocations->locations[i].x<<endl;
    cout<<"humanLocations["<<i<<"].y="<<humanLocations->locations[i].y<<endl;
    cout<<"humanLocations["<<i<<"].z="<<humanLocations->locations[i].z<<endl;
  }
  */

  // using simple MyTracker
  //myTracker.trackHumans(humanLocations);
  //trackedHumansPub.publish(myTracker.getTrackedHumans());

  // using TimTracker
  vector<Tracker::TrackPoint> trackPoints;
  for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    Tracker::TrackPoint point = {humanLocations->locations[i].x,
                                 humanLocations->locations[i].y,
                                 0.01,
                                 0.01};
    trackPoints.push_back(point);
  }
  double deltaTime=0;
  ros::Time now=ros::Time::now();
  ros::Duration duration=now-prevNow;
  deltaTime=duration.toSec();
  prevNow=now;
  tracker.update(trackPoints, deltaTime);

}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "HumanTracker");
  prevNow=ros::Time::now();

  // create publisher and subscribers
  ros::NodeHandle n;
  trackedHumansPub=n.advertise<accompany_human_tracker::TrackedHumans>("/trackedHumans",10);
  ros::Subscriber humanLocationsSub=n.subscribe<accompany_human_tracker::HumanLocations>("/humanLocations",10,humanLocationsReceived);
  
  // read human indentities

  // do tracking

  // publish tracked humans

  ros::spin(); // wait for shutdown

  return 0;
}
