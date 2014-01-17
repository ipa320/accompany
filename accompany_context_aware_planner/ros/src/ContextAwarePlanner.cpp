#include "accompany_context_aware_planner/proxemics.h"
#include <ros/ros.h>

//RH's Setting
//#define DBHOST "tcp://localhost:3306"
//#define DBHOST "tcp://10.0.1.54:3306" //robothouse database ip
//#define USER "rhUser"
//#define PASSWORD "waterloo"
//#define DATABASE "Accompany"


//Heerlen's Setting
#define DBHOST "tcp://192.168.1.142:3306" //robothouse database ip
#define USER "accompanyUser"
#define PASSWORD "accompany"
#define DATABASE "Accompany"


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "context_aware_planner_server");
  ros::NodeHandle n;
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
