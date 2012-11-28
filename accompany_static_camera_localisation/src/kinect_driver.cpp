
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <accompany_uva_utils/uva_utils.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

string configDescriptionFilename;
string configFilename;

void callbackConfigDescription(const dynamic_reconfigure::ConfigDescription::ConstPtr& msg)
{
  cout<<"saving '"<<configDescriptionFilename<<"'"<<endl;
  save_msg(*msg,configDescriptionFilename);
}

void callbackConfig(const dynamic_reconfigure::Config::ConstPtr& msg)
{
  cout<<"saving '"<<configFilename<<"'"<<endl;
  save_msg(*msg,configFilename);
}


int main(int argc, char** argv)
{
  string path;

  // handling arguments
  po::options_description optionsDescription(
      "Saves and republishes the kinect driver configuration data");
  optionsDescription.add_options()
    ("help,h", "produce help message")
    ("save,s", "save the data")
    ("republish,r", "load and republished the data")
    ("path,p",po::value<string >(&path)->default_value("./"),"path where data files are saved/loaded");
  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    po::notify(variablesMap);
  }
  catch (const exception& e)
  {
    cerr << "--------------------" << endl;
    cerr << "- " << e.what() << endl;
    cerr << "--------------------" << endl;
    cerr << optionsDescription << endl;
    return 1;
  }
  
  stringstream ss;
  ss<<path<<"/"<<"ConfigDescription.dat"<<endl;
  configDescriptionFilename=ss.str();
  ss.str("");
  ss<<path<<"/"<<"Config.dat"<<endl;
  configFilename=ss.str();

  ros::init(argc, argv, "kinect_driver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  
  if (variablesMap.count("save"))
  {
    ros::Subscriber sub1 = nh.subscribe("/camera/driver/parameter_descriptions", 1, callbackConfigDescription);
    ros::Subscriber sub2 = nh.subscribe("/camera/driver/parameter_updates", 1, callbackConfig);
    ros::spin();
  }
  else if (variablesMap.count("republish"))
  {
    dynamic_reconfigure::ConfigDescription configDescription;
    load_msg(configDescription,configDescriptionFilename);
    ros::Publisher pub1 = nh.advertise<dynamic_reconfigure::ConfigDescription>("/camera/driver/parameter_descriptions", 1);
    cout<<"publishing '"<<configDescriptionFilename<<"'"<<endl;
    pub1.publish(configDescription);

    dynamic_reconfigure::Config config;
    load_msg(config,configFilename);
    ros::Publisher pub2 = nh.advertise<dynamic_reconfigure::Config>("/camera/driver/parameter_updates", 1);
    cout<<"publishing '"<<configFilename<<"'"<<endl;
    pub1.publish(config);

    ros::spinOnce();
  }

  return 0;
}
