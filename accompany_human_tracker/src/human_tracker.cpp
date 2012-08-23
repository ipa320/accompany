
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHumans.h>
#include <cob_people_detection_msgs/PeopleDetection.h>
#include <cob_people_detection_msgs/PeopleDetectionArray.h>
#include <tf/transform_listener.h>

#include <MyTracker.h>
#include <TimTracker/Tracker.h>

#include <limits>
#include <iostream>
using namespace std;

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


// receive human locations and track them
void humanLocationsReceived(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations)
{
#if TRACKER == MY_TRACKER // using simple MyTracker
 
  myTracker.trackHumans(humanLocations);
  trackedHumans=myTracker.getTrackedHumans();

#elif TRACKER == TIM_TRACKER // using TimTracker
  
  vector<Tracker::TrackPoint> trackPoints;
  for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    Tracker::TrackPoint point = {humanLocations->locations[i].x,
                                 humanLocations->locations[i].y,
                                 0.01,
                                 0.01};
    trackPoints.push_back(point);
  }
  double deltaTime=0;
  ros::Time now=ros::Time::now();
  ros::Duration duration=now-prevNow;
  deltaTime=duration.toSec();
  prevNow=now;
  cout<<"trackPoints.size(): "<<trackPoints.size()<<endl;
  tracker.update(trackPoints, deltaTime);
  cout<<"tracks.size(): "<<tracker.tracks.size()<<endl;
  trackedHumans.trackedHumans.clear();
  for (vector<Tracker::Track>::iterator it=tracker.tracks.begin();it!=tracker.tracks.end();it++)
  {
    cv::Mat mat=it->filter.getState();
    accompany_human_tracker::TrackedHuman trackedHuman;
    trackedHuman.location.x=mat.at<float>(0,0);
    trackedHuman.location.y=mat.at<float>(1,0);
    trackedHuman.location.z=0;
    trackedHuman.id=it->id;
    map<int,string>::iterator it=idToIdentity.find(trackedHuman.id);
    if (it!=idToIdentity.end())
      trackedHuman.identity=it->second;
    trackedHumans.trackedHumans.push_back(trackedHuman);
  }
#endif

  trackedHumansPub.publish(trackedHumans);
}


class MakeMatch
{
public:

  MakeMatch()
  {
    bestSumDist=numeric_limits<double>::max();
  }

  void match(double *dist,int nrTracks,int nrIdentities,vector<int> *assigned,double sumDist)
  {
    if (assigned->size()==(unsigned)nrIdentities)
    {
      if (sumDist<bestSumDist)
      {
        bestSumDist=sumDist;
        bestAssigned=*assigned;
      }
    }
    else
    {
      int iden=assigned->size();// identity to assign
      for (int i=0;i<nrTracks;i++)
      {
        vector<int> newAssigned=*assigned;
        newAssigned.push_back(i);
        double newSumDist=sumDist+dist[i*nrIdentities+iden];
        match(dist,nrTracks,nrIdentities,&newAssigned,newSumDist);
      }
    }
  }

  void match(double *dist,int nrTracks,int nrIdentities)
  {
    vector<int> assigned;
    match(dist,nrTracks,nrIdentities,&assigned,0);
  }

  vector<int> &getBestAssigned()
  {
    return bestAssigned;
  }

  vector<int> bestAssigned;
  double bestSumDist;
};

// compute the distance matrix of every track to every identified human and match using MakeMatch class
void match(cob_people_detection_msgs::PeopleDetectionArray &transformedIdentifiedHumans)
{
  unsigned int nrTracks=trackedHumans.trackedHumans.size();
  unsigned int nrIdentities=transformedIdentifiedHumans.detections.size();
  double dist[nrTracks*nrIdentities];
  for (unsigned int i=0;i<nrTracks;i++)
  {
    accompany_human_tracker::TrackedHuman trackedHuman=trackedHumans.trackedHumans[i];
    for (unsigned int j=0;j<nrIdentities;j++)
    {
      cob_people_detection_msgs::PeopleDetection identity=transformedIdentifiedHumans.detections[j];

      double dx=trackedHuman.location.x-identity.pose.pose.position.x;
      double dy=trackedHuman.location.y-identity.pose.pose.position.y;
      double dz=trackedHuman.location.y-identity.pose.pose.position.z;
      dist[i*nrIdentities+j]=dx*dx+dy*dy+dz*dz;
    }
  }
  
  MakeMatch makeMatch;
  makeMatch.match(dist,nrTracks,nrIdentities);
  vector<int> assigned=makeMatch.getBestAssigned(); // best assignment
  cerr<<"assigned: ";
  int i=0;
  for (vector<int>::iterator it=assigned.begin();it!=assigned.end();it++)
  {
    identityToID[transformedIdentifiedHumans.detections[i++].label]=*it;
    cerr<<*it<<" ";
  }
  cerr<<endl;
  idToIdentity.clear(); //map of id's to identities
  for (map<string,int>::iterator it=identityToID.begin();it!=identityToID.end();it++)
  {
    cerr<<it->first<<"="<<it->second<<endl;
    idToIdentity[it->second]=it->first;
  }
}

// receive identities and transform them to the camera's coordinate system
void identityReceived(const cob_people_detection_msgs::PeopleDetectionArray::ConstPtr& identifiedHumans)
{
  
  cob_people_detection_msgs::PeopleDetectionArray transformedIdentifiedHumans=*identifiedHumans;
  for (unsigned int i=0;i<identifiedHumans->detections.size();i++)
  {
    string identity=identifiedHumans->detections[i].label;
    const geometry_msgs::PoseStamped pose=identifiedHumans->detections[i].pose;

    // transform to camera coordinate system
    
    try
    {
      geometry_msgs::PoseStamped transPose;
      

      listener->transformPose("/overhead1",
                              pose,
                              transPose);
      geometry_msgs::Point pos=transPose.pose.position;
      cerr<<"identified '"<<identity<<"' at ("<<pos.x<<","<<pos.y<<","<<pos.z<<")"<<endl;
      transformedIdentifiedHumans.detections[i].pose=transPose;
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human identity to overhead camera frame: "<<e.what()<<endl;
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
  ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::PeopleDetectionArray>("/face_recognitions",10,identityReceived);
  ros::spin();

  return 0;
}
