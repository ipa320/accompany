#ifndef Tracker_INCLUDED
#define Tracker_INCLUDED

#include <Track.h>
#include <DataAssociation.h>

#include <ros/ros.h>
#include <accompany_uva_msg/MsgToMarkerArray.h>

#include <vector>

class Tracker
{
 public:
  Tracker(ros::Publisher trackedHumansPub,
          ros::Publisher markerArrayPub);
  void processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections);
  
  friend std::ostream& operator<<(std::ostream& out,const Tracker& tracker);

 private:
  std::vector<Track> tracks;
  DataAssociation dataAssociation;

  vnl_matrix<double>
    transModel, transCovariance,
    obsModel,   obsCovariance;

  struct timeval prevTime;

  ros::Publisher trackedHumansPub;
  MsgToMarkerArray msgToMarkerArray;
  ros::Publisher markerArrayPub;
  void publishTracks();

};

#endif
