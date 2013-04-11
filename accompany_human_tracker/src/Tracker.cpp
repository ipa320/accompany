#include <Tracker.h>

using namespace std;

/**
 * Constructor
 * @param trackedHumansPub 'trackedHuman' publisher
 * @param markerArrayPub 'markerArray' publisher for visualization purposes
 * @param transformListener used to transform to world coordinates
 */
Tracker::Tracker(const ros::Publisher& trackedHumansPub,
                 const ros::Publisher& markerArrayPub)
{
  this->trackedHumansPub=trackedHumansPub;
  this->markerArrayPub=markerArrayPub;

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
  // transform to world coordinates
  accompany_uva_msg::HumanDetections transHumanDetections=transform(*humanDetections);

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
  dataAssociation.clear(tracks.size(),transHumanDetections.detections.size());
  for (unsigned i=0;i<tracks.size();i++)
    for (unsigned j=0;j<transHumanDetections.detections.size();j++)
      dataAssociation.set(i,j,tracks[i].match(transHumanDetections.detections[j],obsModel));

  vector<int> associations=dataAssociation.associate();

  cout<<"associations:"<<endl;
  for (unsigned i=0;i<associations.size();i++)
    cout<<associations[i]<<" ";
  cout<<endl;

  // update or create tracks
  for (unsigned i=0;i<associations.size();i++)
  {
    if (associations[i]<0) // not assigned
    {
      tracks.push_back(Track(transHumanDetections.detections[i]));
    }
    else // assigned
    {
      tracks[associations[i]].observation(transHumanDetections.detections[i],
                                          obsModel,
                                          obsCovariance);
    }
  }
  
  cout<<*this<<endl;
  prevTime=time;
  
  publishTracks();
}

/**
 * Transform human detections to world coordinates
 */
accompany_uva_msg::HumanDetections Tracker::transform(const accompany_uva_msg::HumanDetections& humanDetections)
{
  accompany_uva_msg::HumanDetections transformedHumanDetections;
  for (unsigned i=0;i<humanDetections.detections.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::PointStamped transPoint;
      transformListener.transformPoint("/map",
                                       humanDetections.detections[i].location,
                                       transPoint);
      transformedHumanDetections.detections.push_back(humanDetections.detections[i]);
      transformedHumanDetections.detections.back().location=transPoint;
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human location: "<<e.what()<<endl;
      break;
    }
  }
  return transformedHumanDetections;
}

/**
 * Publish the known detections.
 */
void Tracker::publishTracks()
{
  accompany_uva_msg::TrackedHumans trackedHumans;
  accompany_uva_msg::TrackedHuman trackedHuman;
  trackedHuman.location.header.stamp=ros::Time::now();
  trackedHuman.location.header.frame_id="/map";
  for (vector<Track>::iterator it=tracks.begin();it!=tracks.end();it++)
  {
    it->writeMessage(trackedHuman);
    /*
    map<int,string>::iterator it=idToIdentity.find(trackedHuman.id);
    if (it!=idToIdentity.end())
      trackedHuman.identity=it->second;
    else
      trackedHuman.identity="";
    */
    trackedHumans.trackedHumans.push_back(trackedHuman);
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
