
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHumans.h>
#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <tf/transform_listener.h>

#include <MyTracker.h>
#include <TimTracker/Tracker.h>
#include <DataAssociation.h>

#include <boost/program_options.hpp>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <limits>
#include <iostream>
using namespace std;
using namespace boost;

#define FUSE_HUMAN_DETECTION_DISTANCE 3 // meters

#define MY_TRACKER	1
#define TIM_TRACKER	2
#ifndef TRACKER
  #define TRACKER TIM_TRACKER // select one of the trackers above
#endif

// globals
ros::Publisher trackedHumansPub;
ros::Time prevNow;
tf::TransformListener *listener=NULL;
map<string,int> identityToID; // map of identities to id's
map<int,string> idToIdentity; // map of id's to identities

accompany_human_tracker::TrackedHumans trackedHumans;// array of tracked humans
#if TRACKER == MY_TRACKER
MyTracker myTracker;
#elif TRACKER == TIM_TRACKER
Tracker tracker;
#endif

// map to store all TrackPoints accesible by the name of the coordinate frame
typedef std::map<string,vector<Tracker::TrackPoint> > TrackPointsMap;
TrackPointsMap trackPointsMap;

struct TrackPointsTraits // used to access all elements 
{
  static unsigned int getSize(const vector<Tracker::TrackPoint> &t)
  {
    return t.size();
  }

  static const Tracker::TrackPoint &getElement(const vector<Tracker::TrackPoint> &t,int i)
  {
    return t[i];
  }

};

struct DistanceTrackPoint // computes distance between elements
{
  static double squareDistance(const Tracker::TrackPoint &t1,
                               const Tracker::TrackPoint &t2)
  {
    double dx=t1.x-t2.x;
    double dy=t1.y-t2.y;
    return dx*dx+dy*dy;
  }
};

DataAssociation<vector<Tracker::TrackPoint>,TrackPointsTraits,
                vector<Tracker::TrackPoint>,TrackPointsTraits,
                DistanceTrackPoint> trackPointsDataAssociation;


vector<Tracker::TrackPoint> humanLocationsToTrackpoints(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations)
{
  vector<Tracker::TrackPoint> trackPoints;
  for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::Vector3Stamped transVec;
      listener->transformVector("/map",
                                humanLocations->locations[i],
                                transVec);
      Tracker::TrackPoint point = {transVec.vector.x,
                                   transVec.vector.y,
                                   0.01,
                                   0.01};
      trackPoints.push_back(point);
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human location: "<<e.what()<<endl;
      break;
    }
  }
  return trackPoints;
}

void fuseTracks(vector<Tracker::TrackPoint> &trackPoints,TrackPointsMap trackPointsMap)
{
  bool first=true;
  BOOST_FOREACH(TrackPointsMap::value_type &it, trackPointsMap) 
  {
    cout<<"observations from "<<it.first<<endl;
    if (first)
    {
      first=false;
      trackPoints=it.second;
    }
    else
    {
      set<int> indices;
      for (unsigned int i=0;i<it.second.size();i++) // get all indices
        indices.insert(i);
      trackPointsDataAssociation.buildDistanceMatrix(trackPoints,it.second);
      double distance;
      int index1,index2;
      while (trackPointsDataAssociation.bestMatch(distance,index1,index2))
      {
        if (distance>FUSE_HUMAN_DETECTION_DISTANCE)
          break;
        indices.erase(index2); // match so remote index
        cout<<"  fuse: "<<index2<<endl;
        trackPoints[index1].x=(trackPoints[index1].x+it.second[index2].x)/2;
        trackPoints[index1].y=(trackPoints[index1].y+it.second[index2].y)/2;
      }
      for (unsigned int i=0;i<indices.size();i++) //  add remaining indices to trackPoints
      {
        trackPoints.push_back(it.second[i]);
        cout<<"  add: "<<i<<endl;
      }
    }
  }
}

void updateTrackedHumans()
{
  trackedHumans.trackedHumans.clear();
  accompany_human_tracker::TrackedHuman trackedHuman;
  trackedHuman.location.header.stamp=ros::Time::now();
  trackedHuman.location.header.frame_id="/map";
  for (vector<Tracker::Track>::iterator it=tracker.tracks.begin();it!=tracker.tracks.end();it++)
  {
    cv::Mat mat=it->filter.getState();
    trackedHuman.location.vector.x=mat.at<float>(0,0);
    trackedHuman.location.vector.y=mat.at<float>(1,0);
    trackedHuman.location.vector.z=0;
    trackedHuman.id=it->id;
    map<int,string>::iterator it=idToIdentity.find(trackedHuman.id);
    if (it!=idToIdentity.end())
      trackedHuman.identity=it->second;
    else
      trackedHuman.identity="";
    trackedHumans.trackedHumans.push_back(trackedHuman);
  }
}

// receive human locations and track them
void humanLocationsReceived(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations)
{
#if TRACKER == MY_TRACKER // using simple MyTracker

  myTracker.trackHumans(humanLocations);
  trackedHumans=myTracker.getTrackedHumans();

#elif TRACKER == TIM_TRACKER // using TimTracker

  if (humanLocations->locations.size()>0)
  {
    string frameName=humanLocations->locations[0].header.frame_id;
    trackPointsMap[frameName]=humanLocationsToTrackpoints(humanLocations);
    vector<Tracker::TrackPoint> fusedTrackPoints;
    fuseTracks(fusedTrackPoints,trackPointsMap);
    
    double deltaTime=0;
    ros::Time now=ros::Time::now();
    ros::Duration duration=now-prevNow;
    deltaTime=duration.toSec();
    prevNow=now;
    cout<<"trackPoints.size(): "<<fusedTrackPoints.size()<<endl;
    tracker.update(fusedTrackPoints, deltaTime);
    cout<<"tracks.size(): "<<tracker.tracks.size()<<endl;
    updateTrackedHumans();
  }
#endif

  trackedHumansPub.publish(trackedHumans);
}

struct TrackedHumansTraits // used to access all elements 
{
  static unsigned int getSize(const accompany_human_tracker::TrackedHumans &t)
  {
    return t.trackedHumans.size();
  }

  static const geometry_msgs::Vector3 &getElement(const accompany_human_tracker::TrackedHumans &t,int i)
  {
    return t.trackedHumans[i].location.vector;
  }

};

struct DetectionArrayTraits // used to access all elements 
{
  static unsigned int getSize(const cob_people_detection_msgs::DetectionArray &t)
  {
    return t.detections.size();
  }

  static const geometry_msgs::Point &getElement(const cob_people_detection_msgs::DetectionArray &t,int i)
  {
    return t.detections[i].pose.pose.position;
  }

};

struct DistanceVector3Point // computes distance between elements
{
  static double squareDistance(const geometry_msgs::Vector3 &t1,
                               const geometry_msgs::Point &t2)
  {
    double dx=t1.x-t2.x;
    double dy=t1.y-t2.y;
    return dx*dx+dy*dy;
  }
};

DataAssociation<accompany_human_tracker::TrackedHumans,TrackedHumansTraits,
                cob_people_detection_msgs::DetectionArray,DetectionArrayTraits,
                DistanceVector3Point> identityDataAssociation;

// compute the distance matrix of every track to every identified human and match using MakeMatch class
void match(cob_people_detection_msgs::DetectionArray &transformedIdentifiedHumans)
{
  identityDataAssociation.buildDistanceMatrix(trackedHumans,transformedIdentifiedHumans);
  double sumDistance;
  vector<std::pair<int,int> > indices=identityDataAssociation.globalBestMatches(sumDistance);
  for (unsigned int i=0;i<indices.size();i++)
  {
    cout<<indices[i].first<<" - "<<indices[i].second<<endl;
    string indentity=transformedIdentifiedHumans.detections[indices[i].second].label;
    identityToID[indentity]=trackedHumans.trackedHumans[indices[i].first].id;
  }
  idToIdentity.clear(); //new map of id's to identities
  for (map<string,int>::iterator it=identityToID.begin();it!=identityToID.end();it++)
    idToIdentity[it->second]=it->first;
}

// receive identities and transform them to the camera's coordinate system
void identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& identifiedHumans)
{
  cob_people_detection_msgs::DetectionArray transformedIdentifiedHumans=*identifiedHumans;
  for (unsigned int i=0;i<identifiedHumans->detections.size();i++)
  {
    string identity=identifiedHumans->detections[i].label;
    const geometry_msgs::PoseStamped pose=identifiedHumans->detections[i].pose;

    try// transform to map coordinate system
    {
      geometry_msgs::PoseStamped transPose;
      listener->transformPose("/map",
                              pose,
                              transPose);
      geometry_msgs::Point pos=transPose.pose.position;
      cerr<<"identified '"<<identity<<"' at ("<<pos.x<<","<<pos.y<<","<<pos.z<<")"<<endl;
      transformedIdentifiedHumans.detections[i].pose=transPose;
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human identity: "<<e.what()<<endl;
      break;
    }
  }
  match(transformedIdentifiedHumans);
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "human_tracker");

  // handling arguments
  program_options::options_description optionsDescription(
      "view_track views human detections and tracks humans");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("graceTime,t", program_options::value<double>(&Tracker::graceTime)->default_value(1.0),"time in seconds to keep a track alive after no more obervations are received");

  program_options::variables_map variablesMap;
  try
  {
    program_options::store(program_options::parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    program_options::notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    cerr<<""<<e.what()<<endl;    
    return 1;
  }

  // create publisher and subscribers
  ros::NodeHandle n;

  prevNow=ros::Time::now();
  tf::TransformListener initListener;
  listener=&initListener;

  trackedHumansPub=n.advertise<accompany_human_tracker::TrackedHumans>("/trackedHumans",10);
  ros::Subscriber humanLocationsSub=n.subscribe<accompany_human_tracker::HumanLocations>("/humanLocations",10,humanLocationsReceived);
  ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10,identityReceived);
  ros::spin();

  return 0;
}
