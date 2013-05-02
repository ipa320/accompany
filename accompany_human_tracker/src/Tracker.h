#ifndef Tracker_INCLUDED
#define Tracker_INCLUDED

#include <Track.h>
#include <DataAssociation.h>

#include <ros/ros.h>
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <tf/transform_listener.h>

#include <vector>

#include <accompany_static_camera_localisation/Hull.h>

/**
 *  Tracks detections based on position and color
 */
class Tracker
{
 public:
  Tracker(const ros::Publisher& trackedHumansPub,
          const ros::Publisher& markerArrayPub,
          const std::vector<WorldPoint>& priorHull,
          const std::vector< std::vector<WorldPoint> >& entryExitHulls,
          double stateThreshold,
          double appearanceThreshold,
          double totalThreshold);
  void processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections);
  
  friend std::ostream& operator<<(std::ostream& out,const Tracker& tracker);

 private:
  std::vector<Track> tracks;

  DataAssociation dataAssociation;
  std::vector<WorldPoint> priorHull;
  std::vector< std::vector<WorldPoint> > entryExitHulls;
  double stateThreshold,appearanceThreshold,totalThreshold;

  vnl_matrix<double>
    transModel, transCovariance,
    obsModel,   obsCovariance;

  struct timeval prevTime;

  std::string coordFrame;
  ros::Publisher trackedHumansPub;
  MsgToMarkerArray msgToMarkerArray;
  ros::Publisher markerArrayPub;
  tf::TransformListener transformListener;

  bool insideEntry(const accompany_uva_msg::HumanDetection& detection);
  accompany_uva_msg::HumanDetections transform(const accompany_uva_msg::HumanDetections& humanDetections);
  void publishTracks();


};

#endif
