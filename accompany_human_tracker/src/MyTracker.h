#ifndef MyTracker_H
#define MyTracker_H

#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHumans.h>
#include <list>


class MyTracker
{
 public:
  MyTracker();

  void trackHumans(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations);
  accompany_human_tracker::TrackedHumans getTrackedHumans();

 private:
  int getID();
  void removeOldTracks();
  void update(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations,int index,
              std::list<accompany_human_tracker::TrackedHuman>::iterator it);
  void add(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations,int index);

  int id;
  std::list<accompany_human_tracker::TrackedHuman> trackedHumansList;
  
};

#endif
