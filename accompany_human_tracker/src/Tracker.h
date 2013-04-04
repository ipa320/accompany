#ifndef Tracker_INCLUDED
#define Tracker_INCLUDED

#include <Track.h>
#include <DataAssociation.h>

#include <vector>

class Tracker
{
 public:
  void processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections);
  
  friend std::ostream& operator<<(std::ostream& out,const Tracker& tracker);

 private:
  std::vector<Track> tracks;
  DataAssociation dataAssociation;

};

#endif
