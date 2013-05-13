
#include <Tracker.h>

#include <boost/program_options.hpp>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <limits>
#include <iostream>
using namespace std;
using namespace boost;


vector<WorldPoint> priorHull;
vector< vector<WorldPoint> > entryExitHulls;

int main(int argc,char **argv)
{
  ros::init(argc, argv, "human_tracker");

  string param_path;
  double stateThreshold,appearanceThreshold,totalThreshold;

  // handling command line arguments
  program_options::options_description optionsDescription("Tracks humans using human detections");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("path,p", program_options::value<string>(&param_path)->required(),"path to pior.txt and entryExit.txt")
    ("stateThreshold,s",program_options::value<double>(&stateThreshold)->default_value(-6),"threshold on the kalman filter state")
    ("appearanceThreshold,a",program_options::value<double>(&appearanceThreshold)->default_value(-0),"threshold on the appearance")
    ("totalThreshold,t",program_options::value<double>(&totalThreshold)->default_value(-6),"combined threshold on state and appearance");

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
  
  string prior_file = param_path + "/" + "prior.txt";
  string entryExit_file = param_path + "/" + "entryExit.txt";
  
  loadHull(prior_file.c_str(),priorHull);
  cout<<"priorHull:"<<endl<<priorHull<<endl;
  loadHulls(entryExit_file.c_str(),entryExitHulls);
  cout<<"entryExitHulls:"<<endl<<entryExitHulls<<endl;

  // create publisher and subscribers
  ros::NodeHandle n;
  ros::Publisher trackedHumansPub=n.advertise<accompany_uva_msg::TrackedHumans>("/trackedHumans",10);
  ros::Publisher markerArrayPub  =n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);

  // create Tracker
  Tracker tracker(trackedHumansPub,markerArrayPub,
                  priorHull,
                  entryExitHulls,
                  stateThreshold,
                  appearanceThreshold,
                  totalThreshold);
  
  // subscribers
  ros::Subscriber humanDetectionsSub=n.subscribe<accompany_uva_msg::HumanDetections>("/humanDetections",10,
                                                                                     &Tracker::processDetections,&tracker);
  ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10,
                                                                                     &Tracker::identityReceived,&tracker);
  ros::Subscriber tfSub= n.subscribe("tf", 100,
                                     &Tracker::tfCallBack,&tracker);
  ros::spin();

  return 0;
}
