#include "accompany_context_aware_planner/proxemics.h"
#include <ros/ros.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "context_aware_planner_server");
  ros::NodeHandle n;

  //Declare MySQL server variables that can be modified by launch file or command line.
  std::string DBHOST;
  std::string USER;
  std::string PASSWORD;
  std::string DATABASE;

  //Initialize node parameters from launch file or command line.
  //Use a private node handle so hat multiple instances of the node can be run simultaneously while using different parameters.
  ros::NodeHandle private_node_handle_("~");

  //Default MySQL setting for local host
  private_node_handle_.param("DBHOST",		DBHOST, 	string("tcp://localhost:3306"));
  private_node_handle_.param("USER",		USER, 		string("rhUser"));
  private_node_handle_.param("PASSWORD",	PASSWORD, 	string("waterloo"));
  private_node_handle_.param("DATABASE",	DATABASE, 	string("Accompany"));

  Proxemics p;
  p.init(n, DBHOST, USER, PASSWORD, DATABASE);
  ros::spin();


/*
  if (argc == 2)
  {
    std::string str = argv[1];
    if (!str.compare("Y1"))
    {
      ros::Rate r(10); // 10 hz
      while (ros::ok())
      {
     //   p.getPotentialProxemicsLocations_Sofa_Y1();
        r.sleep();
      }
    }
    else if (!str.compare("Y2"))
          {

            ros::spin();

          }
  }
  else
    ROS_INFO("usage:    'accompany_context_aware_planner Y1' for year 1 context aware planner implementation");
    ROS_INFO("          'accompany_context_aware_planner Y2' for year 2 context aware planner implementation");
*/
  return 0;
}
