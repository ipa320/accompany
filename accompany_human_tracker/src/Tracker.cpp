#include <Tracker.h>

using namespace std;

/**
 * Constructor
 * @param trackedHumansPub 'trackedHuman' publisher
 * @param markerArrayPub 'markerArray' publisher for visualization purposes
 * @param priorHull the area where people are detected
 * @param entryExitHulls the entry and exit areas of the scene
 * @param stateThreshold matching threshold on distance
 * @param appearanceThreshold matching threshold on appearance
 * @param totalThreshold combined threshold on distance and appearance 
 * @param minMatchCount the minimum number of matches before a track is published
 * @param maxUnmatchCount the maximum number of consecutive non-matches before a track might be removed 
 */
Tracker::Tracker(const ros::Publisher& trackedHumansPub,
                 const ros::Publisher& markerArrayPub,
                 const std::vector<WorldPoint>& priorHull,
                 const std::vector< std::vector<WorldPoint> >& entryExitHulls,
                 double stateThreshold,
                 double appearanceThreshold,
                 double totalThreshold,
                 unsigned minMatchCount,
                 unsigned maxUnmatchCount)
{
  this->trackedHumansPub=trackedHumansPub;
  this->markerArrayPub=markerArrayPub;
  this->priorHull=priorHull;
  this->entryExitHulls=entryExitHulls;
  this->stateThreshold=stateThreshold;
  this->appearanceThreshold=appearanceThreshold;
  this->totalThreshold=totalThreshold;
  this->minMatchCount=minMatchCount;
  this->maxUnmatchCount=maxUnmatchCount;
  // kalman motion and observation model
  const double tm[]={1,0,1,0,
                     0,1,0,1,
                     0,0,1,0,
                     0,0,0,1};
  transModel=vnl_matrix<double>(tm,4,4);
  const double tc[]={1,0,0,0,
                     0,1,0,0,
                     0,0,1,0,
                     0,0,0,1};
  transCovariance=vnl_matrix<double>(tc,4,4);
  const double om[]={1,0,0,0,
                     0,1,0,0};
  obsModel=vnl_matrix<double>(om,2,4);
  const double oc[]={1,0,
                     0,1};
  obsCovariance=vnl_matrix<double>(oc,2,2);
}

WorldPoint toWorldPoint(const accompany_uva_msg::HumanDetection& detection)
{
  WorldPoint wp(detection.location.point.x,
                detection.location.point.y,
                detection.location.point.z);
  wp*=1000.0; // from meters to millimeters
  return wp;
}

double timeDiff(const struct timeval& time,
                const struct timeval& prevTime)
{
  return (time.tv_sec-prevTime.tv_sec)+(time.tv_usec-prevTime.tv_usec)/((double)1E6);
}

/**
 * Processes detections, assign detections to known tracks or create new tracks
 * @param humanDetections the detections
 */
void Tracker::processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections)
{
  if (humanDetections->detections.size()>0)
    coordFrame=humanDetections->detections[0].location.header.frame_id;

  // transition based on elapsed time
  struct timeval time;
  gettimeofday(&time, NULL);
  if (tracks.size()>0)
  {
    double diff=timeDiff(time,prevTime);
    transModel[0][2]=diff; // set time past so the speed vector get multiplied before adding to the position vector
    transModel[1][3]=diff;
  }
  for (unsigned i=0;i<tracks.size();i++)
    tracks[i].transition(transModel,transCovariance);

  // associate observations with tracks
  dataAssociation.clear(tracks.size(),humanDetections->detections.size());
  for (unsigned i=0;i<tracks.size();i++)
    for (unsigned j=0;j<humanDetections->detections.size();j++)
    {
      double match=tracks[i].match(humanDetections->detections[j],
                                   obsModel,
                                   stateThreshold,
                                   appearanceThreshold);
      dataAssociation.set(i,j,match);
    }

  vector<int> associations=dataAssociation.associate(totalThreshold);

  // print associations
  cout<<"associations:"<<endl;
  for (unsigned i=0;i<associations.size();i++)
    cout<<associations[i]<<" ";
  cout<<endl;

  // add unmatch count of tracks
  for (unsigned i=0;i<tracks.size();i++)
    tracks[i].addUnmatchCount();

  // update or create tracks
  for (unsigned i=0;i<associations.size();i++)
  {
    if (associations[i]<0) // not assigned
    {
      if (inside(toWorldPoint(humanDetections->detections[i]),entryExitHulls))
      {
        cout<<"unassociated detection in entryExitHulls, new track started"<<endl;
        tracks.push_back(Track(humanDetections->detections[i]));
      }
      else
        cout<<"unassociated detection NOT in entryExitHulls, ignore"<<endl;
    }
    else // assigned
    {
      tracks[associations[i]].observation(humanDetections->detections[i],
                                          obsModel,
                                          obsCovariance);
    }
  }
  
  removeTracks();
  reduceSpeed();

  cout<<*this<<endl;
  prevTime=time;
  
  publishTracks();
}

void Tracker::identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& detectionArray)
{
  for (unsigned i=0;i<detectionArray->detections.size();i++)
  {
    geometry_msgs::PoseStamped pose=detectionArray->detections[i].pose;
    try// transform to /camera_frame coordinate system
    {
      transformListener.waitForTransform(pose.header.frame_id,coordFrame,pose.header.stamp,ros::Duration(.3));
      geometry_msgs::PoseStamped transPose;
      transformListener.transformPose(coordFrame,
                                      pose,
                                      transPose);
      geometry_msgs::PointStamped transTFPoint;
      transTFPoint.header=transPose.header;
      transTFPoint.point=transPose.pose.position;
      label(transTFPoint,detectionArray->detections[i].label);
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human location: "<<e.what()<<endl;
    }
  }
}

void Tracker::tfCallBack(const tf::tfMessage& tf)
{
  if (tf.transforms[0].child_frame_id=="/base_link")
  {
    try// transform to /camera_frame coordinate system
    {
      geometry_msgs::PointStamped tfPoint;
      tfPoint.header=tf.transforms[0].header;
      tfPoint.point.x=tf.transforms[0].transform.translation.x;
      tfPoint.point.y=tf.transforms[0].transform.translation.y;
      tfPoint.point.z=tf.transforms[0].transform.translation.z;
      transformListener.waitForTransform(tfPoint.header.frame_id,coordFrame,tfPoint.header.stamp,ros::Duration(.3));
      geometry_msgs::PointStamped transTFPoint;
      transformListener.transformPoint(coordFrame,
                                       tfPoint,
                                       transTFPoint);
      label(transTFPoint,"robot");
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human location: "<<e.what()<<endl;
    }
  }
}

void Tracker::label(geometry_msgs::PointStamped point,string label)
{
  WorldPoint wp(point.point.x,
                point.point.y,
                point.point.z);
  wp*=1000.0; // from meters to millimeters
  
}

/**
 * Reduce speed when unmatched so not to move far away from last matching observation
 */
void Tracker::reduceSpeed()
{
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
  {
    cout<<"unmatchedCount:"<<it->unmatchedCount<<endl;
    if (it->unmatchedCount>0) // if not matched in last cycle
    {
      cout<<"reducedSpeed1:"<<*it<<endl;
      it->reduceSpeed();
      cout<<"reducedSpeed2:"<<*it<<endl;
    }
  }
}

/**
 * Remove tracks when unmatched for more than maxUnmatchCount and (in entryExitHulls or not in priorHull);
 */
void Tracker::removeTracks()
{
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();)
  {
    bool remove=false;
    if (it->unmatchedCount>=maxUnmatchCount) // if not matched for some time
    {
      if (inside(it->toWorldPoint(),entryExitHulls)) // if in entryExitHulls
      {
        cout<<"remove track because unmatched="<<it->unmatchedCount<<" and is in entryExitHulls"<<endl;
        tracks.erase(it);
        remove=true;
      }
      if (!inside(it->toWorldPoint(),priorHull)) // if not in priorHull
      {
        cout<<"remove track because unmatched="<<it->unmatchedCount<<" and is out of priorHull"<<endl;
        tracks.erase(it);
        remove=true;
      }
    }
    if (!remove) it++;
  }
}

/**
 * Publish the known tracks.
 */
void Tracker::publishTracks()
{
  accompany_uva_msg::TrackedHumans trackedHumans;
  accompany_uva_msg::TrackedHuman trackedHuman;
  trackedHuman.location.header.stamp=ros::Time::now();
  trackedHuman.location.header.frame_id=coordFrame;
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
  {
    if (it->matchCount>=minMatchCount)
    {
      it->writeMessage(trackedHuman);
      trackedHumans.trackedHumans.push_back(trackedHuman);
      /*
        map<int,string>::iterator it=idToIdentity.find(trackedHuman.id);
        if (it!=idToIdentity.end())
        trackedHuman.identity=it->second;
        else
        trackedHuman.identity="";
      */
    }
  }
  trackedHumansPub.publish(trackedHumans);
  markerArrayPub.publish(msgToMarkerArray.toMarkerArray(trackedHumans,"trackedHumans")); // publish visualisation
}


/**
 * ostream a tracker
 */
std::ostream& operator<<(std::ostream& out,const Tracker& tracker)
{
  out<<"Tracker ("<<tracker.tracks.size()<<"):"<<endl;
  out<<tracker.dataAssociation;
  /*
  for (unsigned i=0;i<tracker.tracks.size();i++)
    out<<"["<<i<<"] "<<tracker.tracks[i];
  out<<endl;
  */
  return out;
}
