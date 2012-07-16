#ifndef MyTracker_H
#define MyTracker_H

#include <HumanTracker/HumanLocations.h>
#include <HumanTracker/TrackedHumans.h>
#include <list>


class MyTracker
{
 public:
  MyTracker();

  void trackHumans(const HumanTracker::HumanLocations::ConstPtr& humanLocations);
  HumanTracker::TrackedHumans getTrackedHumans();

 private:
  int getID();
  void removeOldTracks();
  void update(const HumanTracker::HumanLocations::ConstPtr& humanLocations,int index,
              std::list<HumanTracker::TrackedHuman>::iterator it);
  void add(const HumanTracker::HumanLocations::ConstPtr& humanLocations,int index);

  int id;
  std::list<HumanTracker::TrackedHuman> trackedHumansList;
  
};

#endif
