#include <Tracker.h>

using namespace std;

double humenProbMultiplier=0.95; // decay
double humenProbAddition=(1-humenProbMultiplier)/humenProbMultiplier;

/**
 * Constructor
 * @param trackedHumansPub 'trackedHuman' publisher
 * @param markerArrayPub 'markerArray' publisher for visualization purposes
 * @param priorHull the area where people are detected
 * @param entryExitHulls the entry and exit areas of the scene
 * @param stateThreshold matching threshold on distance
 * @param appearanceThreshold matching threshold on appearance
 * @param appearanceUpdate weight of new appearance vs old one
 * @param maxSpeed the maximum speed of a track
 * @param minMatchCount the minimum number of matches before a track is published
 * @param maxUnmatchCount the maximum number of consecutive non-matches before a track might be removed 
 */
Tracker::Tracker(const ros::Publisher& trackedHumansPub,
                 const ros::Publisher& markerArrayPub,
                 const std::vector<WorldPoint>& priorHull,
                 const std::vector< std::vector<WorldPoint> >& entryExitHulls,
                 double stateThreshold,
                 double appearanceThreshold,
                 double appearanceUpdate,
                 double maxSpeed,
                 double maxCovar,
                 int minTrackCreateTime,
                 int maxTrackCreateTime,
                 double robotRadius,
                 int nrTracks,
                 unsigned minMatchCount,
                 unsigned maxUnmatchCount)
{
  this->trackedHumansPub=trackedHumansPub;
  this->markerArrayPub=markerArrayPub;
  this->priorHull=priorHull;
  this->entryExitHulls=entryExitHulls;
  this->stateThreshold=stateThreshold;
  this->appearanceThreshold=appearanceThreshold;
  this->appearanceUpdate=appearanceUpdate;
  this->maxSpeed=maxSpeed;
  this->maxCovar=maxCovar;
  this->minTrackCreateTime=minTrackCreateTime;
  this->maxTrackCreateTime=maxTrackCreateTime;
  this->minMatchCount=minMatchCount;
  this->maxUnmatchCount=maxUnmatchCount;
  this->robotRadius=robotRadius;
  this->nrTracks=nrTracks;
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
  const double oc[]={2,0,
                     0,2};
  obsCovariance=vnl_matrix<double>(oc,2,2);
  coordFrame="";// set coordinate frame of HumanDetections to unknown
  trackerCount=0;
  robot=NULL;
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
 * Processes human detections, assign detections to known tracks or create new tracks
 * @param humanDetections the detections
 */
void Tracker::processDetections(const accompany_uva_msg::HumanDetections::ConstPtr& humanDetections)
{
  cout<<"-received "<<humanDetections->detections.size()<<" HumanDetections"<<endl;
  //for (unsigned i=0;i<humanDetections->detections.size();i++)
  //  cout<<humanDetections->detections[i]<<endl;
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
    tracks[i].transition(transModel,transCovariance,maxCovar);

  // associate observations with tracks
  dataAssociation.clear(tracks.size(),humanDetections->detections.size());
  for (unsigned i=0;i<tracks.size();i++)
  {
    for (unsigned j=0;j<humanDetections->detections.size();j++)
    {
      double match=tracks[i].match(humanDetections->detections[j],
                                   obsModel,
                                   stateThreshold,
                                   appearanceThreshold);
      dataAssociation.set(i,j,match);
    }
  }

  //cout<<"dataAssociation:"<<dataAssociation<<endl;
  vector<int> associations=dataAssociation.associate(stateThreshold*appearanceThreshold);

  /*
  // print associations
  cout<<"associations:"<<endl;
  for (unsigned i=0;i<associations.size();i++)
    cout<<associations[i]<<" ";
  cout<<endl;
  */

  // add unmatch count of tracks
  for (unsigned i=0;i<tracks.size();i++)
    tracks[i].addUnmatchCount();

  // update tracks
  int assignedCount=0;
  for (unsigned i=0;i<associations.size();i++)
  {
    if (assignedCount>=nrTracks) break; // max tracks reached
    if (associations[i]>=0) // associated
    {
      vnl_vector<double> speedBefore=tracks[associations[i]].getSpeed();
      tracks[associations[i]].observation(humanDetections->detections[i],
                                          appearanceUpdate,
                                          obsModel,
                                          obsCovariance,
                                          maxCovar);
      assignedCount++;
      cout<<"associated detection, track updated"<<endl;
      vnl_vector<double> speedAfter=tracks[associations[i]].getSpeed();
      double sp=speedAfter.two_norm();
      if (sp>0.2) // has speed?
      {
        tracks[associations[i]].humanProb+=humenProbAddition/2; // more likely a human
        double smoothSp=(speedAfter-speedBefore).two_norm();
        if (smoothSp<0.3) // has constant speed?
        {
          tracks[associations[i]].humanProb+=humenProbAddition/2; // even more likely a human
        }
      }
    }
  }
  // create new tracks
  for (unsigned i=0;i<associations.size();i++)
  {
    if (assignedCount>=nrTracks) break; // max tracks reached 
    if (associations[i]<0) // not associated
    {
      if (inside(toWorldPoint(humanDetections->detections[i]),entryExitHulls) || 
          (trackerCount>=minTrackCreateTime && trackerCount<=maxTrackCreateTime))
      {
        if (tracks.size()<nrTracks)
        {
          tracks.push_back(Track(humanDetections->detections[i],maxCovar));
          assignedCount++;
          cout<<"unassociated detection in entryExitHulls, new track created"<<endl;
        }
        else
        {
          int maxUnmatchedIndex=mostUnmatchedTrackIndex();
          if (maxUnmatchedIndex>=0)
          {
            tracks[maxUnmatchedIndex]=Track(humanDetections->detections[i],maxCovar);
            assignedCount++;
            cout<<"unassociated detection in entryExitHulls, old track overwritten"<<endl;
          }
          else
            cout<<"unassociated detection ignored: NO track slot available"<<endl;
        }
      }
      else
        cout<<"unassociated detection ignored: NOT in entryExitHulls"<<endl;
    }
  }
 
  //cout<<" trackerCount: "<<trackerCount<<" assignedCount: "<<assignedCount<<endl;
  
  removeTracks();
  reduceSpeed();
  humanProbMultiply();
  annotatetracks();

  cout<<*this<<endl;
  prevTime=time;
  
  publishTracks();
  trackerCount++;
}


/**
 * Processes people face dections. Assign names of people to closest track.
 * @param detectionArray people face dections
 */
void Tracker::identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& detectionArray)
{
  cout<<"-face_detection received size:"<<detectionArray->detections.size()<<endl;
  if (coordFrame.size()>0) // if HumanDetections coordinate frame is known
  {
    for (unsigned i=0;i<detectionArray->detections.size();i++)
    {
      geometry_msgs::PoseStamped pose=detectionArray->detections[i].pose;
      cout<<"-face_detection pose: "<<pose.pose.position.x<<","<<pose.pose.position.y<<","<<pose.pose.position.z<<endl;
      try // transform to HumanDetections coordinate system
      {
        geometry_msgs::PoseStamped transPose;
        transformListener.transformPose(coordFrame,
                                        ros::Time(0), // last know transform
                                        pose,
                                        pose.header.frame_id,
                                        transPose);
        cout<<"-face_detection transformed transPose: "<<transPose.pose.position.x<<","<<transPose.pose.position.y<<","<<transPose.pose.position.z<<endl;
        geometry_msgs::PointStamped transTFPoint;
        transTFPoint.header=transPose.header;
        transTFPoint.point=transPose.pose.position;
	cout<<"-face_detection transformed transTFPoint: "<<transTFPoint.point.x<<","<<transTFPoint.point.y<<","<<transTFPoint.point.z<<endl;
        label(transTFPoint,detectionArray->detections[i].label);
      }
      catch (tf::TransformException e)
      {
        cerr<<"error while tranforming human location: "<<e.what()<<endl;
      }
    }
  }
}

/**
 * Processes robot position. Assign name 'robot' to closest track.
 * @param tf tf coordinate frame
 */
void Tracker::tfCallBack(const tf::tfMessage& tf)
{  
  for (unsigned i=0;i<tf.transforms.size();i++)
  {
    if (tf.transforms[i].child_frame_id=="/base_link") // if robot
    {
      cout<<"-received robot position"<<endl;
      if (coordFrame.size()>0) // if HumanDetections coordinate frame is known
      {
        try// transform to HumanDetections coordinate system
        {
          geometry_msgs::PointStamped tfPoint;
          tfPoint.header=tf.transforms[i].header;
          tfPoint.point.x=tf.transforms[i].transform.translation.x;
          tfPoint.point.y=tf.transforms[i].transform.translation.y;
          tfPoint.point.z=tf.transforms[i].transform.translation.z;
          geometry_msgs::PointStamped transTFPoint;
          transformListener.transformPoint(coordFrame,
                                           ros::Time(0), // last know transform
                                           tfPoint,
                                           tfPoint.header.frame_id,
                                           transTFPoint);
          if (robot!=NULL)
            delete robot;
          robot=new Track(transTFPoint,maxCovar);
        }
        catch (tf::TransformException e)
        {
          cerr<<"error while tranforming robot location: "<<e.what()<<endl;
        }
      }
      break;
    }
  }
}

/**
 * Get index of track with highest unmatched count
 * @returns index or one is no unmatched tracks
 */
int Tracker::mostUnmatchedTrackIndex()
{
  int index=-1;
  unsigned maxUnmatchedCount=0;
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (tracks[i].unmatchedCount>maxUnmatchedCount)
    {
      index=i;
      maxUnmatchedCount=tracks[i].unmatchedCount;
    }
  }
  return index;
}

/**
 * Label the track that is closest to point with label.
 * @param point position of object
 * @param label name of object
 */
void Tracker::label(geometry_msgs::PointStamped point,string label)
{
  double maxDistance=numeric_limits<double>::max();
  int best=-1;
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (!tracks[i].isRobot()) // only non-robot tracks
    {
      if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
      {
        vnl_vector<double> pos=tracks[i].getPosition();
        cout<<"face_detect "<<point.point.x<<","<<point.point.y<<"  "<<pos[0]<<","<<pos[1]<<endl;
        double dx=point.point.x-pos[0];
        double dy=point.point.y-pos[1];
        double distance=sqrt(dx*dx+dy*dy);
        cout<<"face_detect distance:"<<distance<<endl;
        if (distance<3 && distance<maxDistance)
        {
          maxDistance=distance;
          best=(int)(i);
        }
      }
    }
  }
  if (best>=0)
  {
    idToName.setIDName(tracks[best].getID(),label);
    cout<<"face_detect label track "<<tracks[best].getID()<<" '"<<label<<"'"<<endl;
  }
}

/**
 * Reduce speed when unmatched so not to move far away from last matching observation
 */
void Tracker::reduceSpeed()
{
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
  {
    it->maxSpeed(maxSpeed);
    if (it->unmatchedCount>0) // if not matched in last cycle
      it->reduceSpeed();
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
      /*
      if (!inside(it->toWorldPoint(),priorHull)) // if not in priorHull
      {
        cout<<"remove track because unmatched="<<it->unmatchedCount<<" and is out of priorHull"<<endl;
        tracks.erase(it);
        remove=true;
      }
      */
    }
    /*
    if (it->unmatchedCount>100*it->matchCount) // remove when not matched for LONG time
    {
      cout<<"remove track because unmatched="<<it->unmatchedCount<<" > matchCount"<<it->matchCount<<endl;
      tracks.erase(it);
      remove=true;
    }
    */
    if (!remove) it++;
  }
}

/**
 * Multiply humanProb so value is between 0 to 1
 */
void Tracker::humanProbMultiply()
{
  for (unsigned i=0;i<tracks.size();i++)
  {
    tracks[i].humanProb*=humenProbMultiplier;
    cout<<"humanProb: "<<tracks[i].humanProb<<endl;
  }
}

void Tracker::annotatetracks()
{
  // annotate robot
  for (unsigned i=0;i<tracks.size();i++)
  {
    if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
    {
      if (robot!=NULL && tracks[i].distanceTo(*robot)<robotRadius) // if close to robot
        tracks[i].setRobot(true);
      else
        tracks[i].setRobot(false);
    }
  }

  // annotate most likely human
  double humanProb=0;
  int maxIndex=-1;
  for (unsigned i=0;i<tracks.size();i++)
  {
    cout<<"track: "<<i<<endl;
    tracks[i].setHuman(false);
    if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
    {
      if (!tracks[i].isRobot()) // not robot
      {
        cout<<"  humanProb: "<<tracks[i].humanProb<<endl;
        if (tracks[i].humanProb>humanProb)
        {
          humanProb=tracks[i].humanProb;
          maxIndex=i;
          cout<<"  maxIndex: "<<maxIndex<<endl;
        }
      }
    }
  }
  if (maxIndex>=0)
    tracks[maxIndex].setHuman(true);
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

  // hack: for removing bubble at robot
  double min_dist = 1e10;
  int index_min_dist = -1;
  for (unsigned i=0;i<tracks.size();i++)
  {
	  if (robot!=NULL)
	  {
		double dist=robot->distanceTo(tracks[i]);
		if (dist < min_dist && dist < 0.8)
		{
			min_dist = dist;
			index_min_dist = i;
		}
	  }
  }
  
  for (unsigned i=0;i<tracks.size();i++)
  {
    trackedHuman.identity=IDToName::unknown;
    trackedHuman.specialFlag=0;
    if (tracks[i].matchCount>=minMatchCount) // only tracks with proper match count
    {
      tracks[i].writeMessage(trackedHuman);
      string name=idToName.getIDName(tracks[i].getID());
      //if (trackedHuman.identity.compare(IDToName::unknown)==0)
        trackedHuman.identity=name;
      if (trackedHuman.identity.compare("")==0)
    	  trackedHuman.identity = "unknown";
      if (i!=index_min_dist) // hack: condition to remove robot bubble
        trackedHumans.trackedHumans.push_back(trackedHuman);
    }
  }
  trackedHumansPub.publish(trackedHumans);
  markerArrayPub.publish(msgToMarkerArray.toMarkerArray(trackedHumans,"trackedHumans")); // publish visualization
  
}

/**
 * ostream a tracker
 */
std::ostream& operator<<(std::ostream& out,const Tracker& tracker)
{
  out<<"Tracker ("<<tracker.tracks.size()<<"):"<<endl;
  out<<tracker.dataAssociation;
  for (unsigned i=0;i<tracker.tracks.size();i++)
    out<<"["<<i<<"] "<<tracker.tracks[i];
  out<<endl;
  return out;
}
