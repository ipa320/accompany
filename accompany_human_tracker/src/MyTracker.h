#ifndef MyTracker_H
#define MyTracker_H

#include <accompany_uva_msg/HumanLocations.h>
#include <accompany_uva_msg/TrackedHumans.h>
#include <list>


class MyTracker
{
 public:
  MyTracker();

  void trackHumans(const accompany_uva_msg::HumanLocations::ConstPtr& humanLocations);
  accompany_uva_msg::TrackedHumans getTrackedHumans();

 private:
  int getID();
  void removeOldTracks();
  void update(const accompany_uva_msg::HumanLocations::ConstPtr& humanLocations,int index,
              std::list<accompany_uva_msg::TrackedHuman>::iterator it);
  void add(const accompany_uva_msg::HumanLocations::ConstPtr& humanLocations,int index);

  int id;
  std::list<accompany_uva_msg::TrackedHuman> trackedHumansList;
  
};

#endif
