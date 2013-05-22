#ifndef Tracker_INCLUDED
#define Tracker_INCLUDED

#include <Track.h>
#include <DataAssociation.h>
#include <IDToName.h>

#include <ros/ros.h>
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <tf/transform_listener.h>

#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>

#include <vector>

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
          double totalThreshold,
          unsigned minMatchCount=10,
          unsigned maxUnmatchCount=20);
  
  void processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections);
  void identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& detectionArray);
  void tfCallBack(const tf::tfMessage& tf);

  friend std::ostream& operator<<(std::ostream& out,const Tracker& tracker);

 private:
  std::vector<Track> tracks;

  DataAssociation dataAssociation;
  std::vector<WorldPoint> priorHull;
  std::vector< std::vector<WorldPoint> > entryExitHulls;
  
  double stateThreshold,appearanceThreshold,totalThreshold;
  unsigned minMatchCount,maxUnmatchCount;

  vnl_matrix<double>
    transModel, transCovariance,
    obsModel,   obsCovariance;

  struct timeval prevTime;

  std::string coordFrame;
  ros::Publisher trackedHumansPub;
  MsgToMarkerArray msgToMarkerArray;
  ros::Publisher markerArrayPub;
  tf::TransformListener transformListener;

  void label(geometry_msgs::PointStamped point,std::string label);
  IDToName idToName;

  void reduceSpeed();
  void removeTracks();
  void publishTracks();

};

#endif

