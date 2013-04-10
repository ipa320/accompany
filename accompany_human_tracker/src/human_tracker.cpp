
#include <Tracker.h>


//#include <cob_people_detection_msgs/Detection.h>
//#include <cob_people_detection_msgs/DetectionArray.h>
#include <tf/transform_listener.h>

//#include <DataAssociation.h>

#include <boost/program_options.hpp>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <limits>
#include <iostream>
using namespace std;
using namespace boost;

// globals

ros::Time prevNow;
tf::TransformListener *listener=NULL;
//map<string,int> identityToID; // map of identities to id's
//map<int,string> idToIdentity; // map of id's to identities

int main(int argc,char **argv)
{
  ros::init(argc, argv, "human_tracker");

  // handling arguments
  program_options::options_description optionsDescription("Tracks humans using human detections");
  optionsDescription.add_options()
    ("help,h","show help message");

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

  ros::Publisher trackedHumansPub=n.advertise<accompany_uva_msg::TrackedHumans>("/trackedHumans",10);
  ros::Publisher markerArrayPub  =n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);
  // create Tracker
  Tracker tracker(trackedHumansPub,markerArrayPub);

  ros::Subscriber humanDetectionsSub=n.subscribe<accompany_uva_msg::HumanDetections>("/humanDetections",10,
                                                                                     &Tracker::processDetections,&tracker);
  //ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10,identityReceived);
  

  ros::spin();

  return 0;
}
