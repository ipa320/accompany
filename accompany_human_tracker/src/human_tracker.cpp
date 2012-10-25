
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHumans.h>
#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <tf/transform_listener.h>

#include <MyTracker.h>
#include <TimTracker/Tracker.h>

#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <limits>
#include <iostream>
using namespace std;
using namespace boost;

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

void printTrackPoint(Tracker::TrackPoint &trackPoint)
{
  cout<<" x:"<<trackPoint.x<<" y:"<<trackPoint.y<<" xVariance:"<<trackPoint.xVariance<<" yVariance:"<<trackPoint.yVariance;
}

// test if trackPoint is within sqaureDistance of any of the points of trackPoints
bool testTrackPoints(vector<Tracker::TrackPoint> &trackPoints,Tracker::TrackPoint &trackPoint,double sqaureDistance)
{
  bool match=false;
  for (vector<Tracker::TrackPoint>::iterator it=trackPoints.begin();it!=trackPoints.end();it++)
  {
    double dx=it->x-trackPoint.x;
    double dy=it->y-trackPoint.y;
    double dist=dx*dx+dy*dy;
    if (dist<sqaureDistance)
    {
      match=true;
      break;
    }
  }
  return match;
}

// fuse trackpoints by not ignoring points that are within sqaureDistance of points of earlier observations
vector<Tracker::TrackPoint> fuseTrackPoints(double sqaureDistance)
{
  vector<Tracker::TrackPoint> trackPoints;
  
  BOOST_FOREACH(TrackPointsMap::value_type &i, trackPointsMap) 
  {
    cout<<"observations from "<<i.first<<endl;
    vector<Tracker::TrackPoint> points=i.second;
    vector<Tracker::TrackPoint> temp;
    // fill temp with points that don't match any in trackPoints
    for (vector<Tracker::TrackPoint>::iterator it=points.begin();it!=points.end();it++)
    {
      cout<<" test";printTrackPoint(*it);
      if (!testTrackPoints(trackPoints,*it,sqaureDistance)) // if not match schedule for addition
      {
        cout<<"  no match"<<endl;
        temp.push_back(*it);
      }
      else
        cout<<"  ignore because of match"<<endl;
    }
    // add temp to trackPoints
    for (vector<Tracker::TrackPoint>::iterator it=temp.begin();it!=temp.end();it++)
    {
      cout<<"add ";printTrackPoint(*it);cout<<endl;
      trackPoints.push_back(*it);
    }
  }
  return trackPoints;
}

// update tracker with fused trackPoints
void updateTracks(accompany_human_tracker::TrackedHumans &trackedHumans)
{
  vector<Tracker::TrackPoint> trackPoints=fuseTrackPoints(1);
  double deltaTime=0;
  ros::Time now=ros::Time::now();
  ros::Duration duration=now-prevNow;
  deltaTime=duration.toSec();
  prevNow=now;
  cout<<"trackPoints.size(): "<<trackPoints.size()<<endl;
  tracker.update(trackPoints, deltaTime);
  cout<<"tracks.size(): "<<tracker.tracks.size()<<endl;
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
    trackPointsMap[frameName]=trackPoints;
    updateTracks(trackedHumans);

  }
#endif

  trackedHumansPub.publish(trackedHumans);
}

// tries all possible matches between identities and tracks an remember the best one
class MakeMatch
{
public:

  MakeMatch()
  {
    bestSumDist=numeric_limits<double>::max();
  }

  void match(double *dist,int nrIdentities,vector<int> &assigned,set<int> &availableTracks,double sumDist)
  {
    int iden=assigned.size();// identity to assign
    if (iden==(unsigned)nrIdentities)
    {
      bestSumDist=sumDist;
      bestAssigned=assigned;
    }
    else
    {
      for (set<int>::iterator it=availableTracks.begin();it!=availableTracks.end();)
      {
        int track=*it;// track to assign
        set<int>::iterator itold=it;it++;
        double newSumDist=sumDist+dist[track*nrIdentities+iden];
        if (newSumDist<bestSumDist)
        {
          availableTracks.erase(itold);// remove track
          assigned.push_back(track);
          // copy set to avoid current iterator invalidation with future erase
          set<int> new_availableTracks=availableTracks;
          match(dist,nrIdentities,assigned,new_availableTracks,newSumDist);

          // undo changes to assigned and availableTracks for next assignment
          assigned.pop_back();
          availableTracks.insert(track);
        }
      }
    }
  }

  void match(double *dist,int nrTracks,int nrIdentities)
  {
    vector<int> assigned;// empty list of assigned identities
    set<int> availableTracks;
    for (int i=0;i<nrTracks;i++)
      availableTracks.insert(i); // all tracks to assign to
    match(dist,nrIdentities,assigned,availableTracks,0);
  }

  vector<int> *getBestAssigned()
  {
    return &bestAssigned;
  }

private:
  vector<int> bestAssigned;
  double bestSumDist;
};

// compute the distance matrix of every track to every identified human and match using MakeMatch class
void match(cob_people_detection_msgs::DetectionArray &transformedIdentifiedHumans)
{
  unsigned int nrTracks=trackedHumans.trackedHumans.size();
  unsigned int nrIdentities=transformedIdentifiedHumans.detections.size();
  double dist[nrTracks*nrIdentities];
  for (unsigned int i=0;i<nrTracks;i++)
  {
    accompany_human_tracker::TrackedHuman trackedHuman=trackedHumans.trackedHumans[i];
    for (unsigned int j=0;j<nrIdentities;j++)
    {
      cob_people_detection_msgs::Detection identity=transformedIdentifiedHumans.detections[j];

      double dx=trackedHuman.location.vector.x-identity.pose.pose.position.x;
      double dy=trackedHuman.location.vector.y-identity.pose.pose.position.y;
      double dz=trackedHuman.location.vector.z-identity.pose.pose.position.z;
      dist[i*nrIdentities+j]=dx*dx+dy*dy+dz*dz;
    }
  }

  MakeMatch makeMatch;
  makeMatch.match(dist,nrTracks,nrIdentities);
  vector<int> *assigned=makeMatch.getBestAssigned(); // best assignment
  int i=0;
  for (vector<int>::iterator it=assigned->begin();it!=assigned->end();it++)
  {
    int id=*it;
    string indentity=transformedIdentifiedHumans.detections[i++].label;
    identityToID[indentity]=id;
  }
  idToIdentity.clear(); //new map of id's to identities
  for (map<string,int>::iterator it=identityToID.begin();it!=identityToID.end();it++)
    idToIdentity[it->second]=it->first;


  for (map<string,int>::iterator it=identityToID.begin();it!=identityToID.end();it++)
  {
    cerr<<it->first<<"="<<it->second<<" ";
  }
  cerr<<endl;
  for (map<int,string>::iterator it=idToIdentity.begin();it!=idToIdentity.end();it++)
  {
    cerr<<it->first<<"="<<it->second<<" ";
  }
  cerr<<endl;

  //if (idToIdentity.size()!=3 || identityToID.size()!=3)
  //exit(1);
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
