
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

tf::TransformListener *listener=NULL;
tf::TransformBroadcaster *transformBroadcasterPtr=NULL;
geometry_msgs::TransformStamped transformStamped;

string parent;
string frameName;

void printTransformStamped(const geometry_msgs::TransformStamped &transformStamped)
{
  cout<<transformStamped;
  geometry_msgs::Quaternion rot=transformStamped.transform.rotation;
  tf::Matrix3x3 mat(tf::Quaternion(rot.x,rot.y,rot.z,rot.w));
  cout<<"conversion to 3x3 rotation matrix:"<<endl;
  for (int i=0;i<3;i++)    
    cout<<"|"<<setw(12)<<mat[i].x()<<" "<<setw(12)<<mat[i].y()<<" "<<setw(12)<<mat[i].z()<<"|"<<endl;
}

void timerCallback1(const ros::TimerEvent& timerEvent)
{
  // publish transform
  transformStamped.header.stamp=ros::Time::now();
  transformBroadcasterPtr->sendTransform(transformStamped);
  // also publish transform with time in the future so that tf always has a 'current' transform
  transformStamped.header.stamp=ros::Time::now()+ros::Duration(2);
  transformBroadcasterPtr->sendTransform(transformStamped);
  cout<<endl<<"--- publish tf transform"<<endl<<transformStamped;
}

void timerCallback2(const ros::TimerEvent& timerEvent)
{ 
  {
  geometry_msgs::Vector3Stamped vec;
  vec.header.stamp=ros::Time::now();
  vec.header.frame_id=frameName;
  vec.vector.x=10;
  vec.vector.y=5;
  vec.vector.z=0;
  cout<<endl<<"--- transform Vector3Stamped (NO TRANSLATION BECAUSE OF ASSUMED TO BE FREE FLOWING): "<<endl<<vec;
  try// transform to map coordinate system
  {
    geometry_msgs::Vector3Stamped transVec;
    listener->transformVector(parent,
                              vec,
                              transVec);
    cout<<transVec;
  }
  catch (tf::TransformException e)
  {
    cerr<<"error while tranforming human location: "<<e.what()<<endl;
  }
  }

  {
  geometry_msgs::PoseStamped vec;
  vec.header.stamp=ros::Time::now();
  vec.header.frame_id=frameName;
  vec.pose.position.x=10;
  vec.pose.position.y=5;
  vec.pose.position.z=0;
  vec.pose.orientation.w=1;
  cout<<endl<<"--- transform PoseStamped: "<<endl<<vec;
  try// transform to map coordinate system
  {
    geometry_msgs::PoseStamped transVec;
    
    listener->transformPose(parent,
                            vec,
                            transVec);
    cout<<transVec;
  }
  catch (tf::TransformException e)
  {
    cerr<<"error while tranforming human location: "<<e.what()<<endl;
  }
  }

  {
  geometry_msgs::PointStamped vec;
  vec.header.stamp=ros::Time::now();
  vec.header.frame_id=frameName;
  vec.point.x=10;
  vec.point.y=5;
  vec.point.z=0;
  cout<<endl<<"--- transform PointStamped: "<<endl<<vec;
  try// transform to map coordinate system
  {
    geometry_msgs::PointStamped transVec;
    
    listener->transformPoint(parent,
                             vec,
                             transVec);
    cout<<transVec;
  }
  catch (tf::TransformException e)
  {
    cerr<<"error while tranforming human location: "<<e.what()<<endl;
  }
  }

}

int main(int argc, char **argv)
{
  string filename;
  double a;
  double x,y,z;

  // handling arguments
  options_description optionsDescription(
      "Create a coordinate frame relative to a parent and save it to disk\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("frame-name,n", value<string>(&frameName)->default_value("/child_frame"),"name of the new frame")
    ("parent-name,p", value<string>(&parent)->default_value("/map"),"name of parent frame")
    ("angle,a", value<double>(&a)->default_value(0.0),"angle in xy plane in degrees")
    ("xpos,x", value<double>(&x)->default_value(0.0),"x position")
    ("ypos,y", value<double>(&y)->default_value(0.0),"y position")
    ("zpos,z", value<double>(&z)->default_value(0.0),"z position");
    

  variables_map variablesMap;
  try
  {
    store(parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }


  

  
  // create transform message
  tf::Transform transform=tf::Transform(tf::Matrix3x3( cos(a*M_PI/180),sin(a*M_PI/180),0, // rotation matrix
                                                    -sin(a*M_PI/180),cos(a*M_PI/180),0,
                                                     0              ,0              ,1),  
                                        tf::Vector3(x,y,z)); // translation vector
  tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                             ros::Time(0),  // time, not used here
                                                             parent,        // parent coordinate frame
                                                             frameName); // child coordinate frame
  tf::transformStampedTFToMsg(stampedTransform,transformStamped);
  printTransformStamped(transformStamped);
  
  
  // ROS node, subscribers and publishers
  ros::init(argc, argv, "test_transform");
  ros::NodeHandle n;
  tf::TransformBroadcaster transformBroadcaster;
  transformBroadcasterPtr=&transformBroadcaster;
  tf::TransformListener initListener;
  listener=&initListener;
  ros::Timer timer1=n.createTimer(ros::Duration(1),timerCallback1);
  ros::Timer timer2=n.createTimer(ros::Duration(1),timerCallback2);
  ros::spin();

  
  return 0;
}
