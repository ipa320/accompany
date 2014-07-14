
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>

#include <iostream>
using namespace std;
using namespace boost;

int main(int argc,char **argv)
{
  ros::init(argc, argv, "face_detect");

  string name,frame;
  double x,y,z;

  // handling command line arguments
  program_options::options_description optionsDescription("Tracks humans using human detections");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("name,n",program_options::value<string>(&name)->default_value("MyName"),"name of detected person")
    ("frame,f",program_options::value<string>(&frame)->default_value("base_link"),"name of tf coordinate frame")
    ("x-coord,x",program_options::value<double>(&x)->default_value(2),"x-coordinate of detected person")
    ("y-coord,y",program_options::value<double>(&y)->default_value(0),"y-coordinate of detected person")
    ("z-coord,z",program_options::value<double>(&z)->default_value(0),"z-coordinate of detected person");
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
  ros::Publisher faceDetectionPub=n.advertise<cob_people_detection_msgs::DetectionArray>
    ("/cob_people_detection/detection_tracker/face_position_array",1);

  cob_people_detection_msgs::DetectionArray detectionArray;
  detectionArray.header.stamp=ros::Time::now();
  detectionArray.header.frame_id=frame;

  cob_people_detection_msgs::Detection detection;
  detection.header=detectionArray.header;
  detection.label=name;
  detection.pose.header=detectionArray.header;
  detection.pose.pose.position.x=x;
  detection.pose.pose.position.y=y;
  detection.pose.pose.position.z=z;
  detection.pose.pose.orientation.x=0;
  detection.pose.pose.orientation.y=0;
  detection.pose.pose.orientation.z=0;
  detection.pose.pose.orientation.w=1;
  
  detectionArray.detections.push_back(detection);
  
  /*
  faceDetectionPub.publish(detectionArray);
  cout<<"wait some time to connect to roscore"<<endl;
  usleep(5e5);
  cout<<"------- publish: "<<endl<<detectionArray<<endl<<"-------"<<endl;
  ros::spinOnce();
  cout<<"wait some time for the message to get delivered"<<endl;
  usleep(5e5);
  */
  
  // /*
  int nrPublish=3;
  ros::Rate loop_rate(1);
  int count=0;
  while (ros::ok())
  {
    if (count==nrPublish/2)
    {
      faceDetectionPub.publish(detectionArray);
      cout<<"publish: "<<detectionArray<<endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if (count>=nrPublish) break;
  }
  // */

  return 0;
}
