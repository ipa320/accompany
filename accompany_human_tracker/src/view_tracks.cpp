
#include <ros/ros.h>
#include <accompany_human_tracker/TrackedHumans.h>

void trackedHumansReceived(const accompany_human_tracker::TrackedHumans::ConstPtr& trackedHumans)
{
  
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "view_tracks");

  // create publisher and subscribers
  ros::NodeHandle n;

  ros::Subscriber trackedHumansSub=n.subscribe<accompany_human_tracker::TrackedHumans>("/trackedHumans",10,trackedHumansReceived);
  ros::spin();

  return 0;
}
