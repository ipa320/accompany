#ifndef PROXEMICS_H
#define PROXEMICS_H

#include "accompany_context_aware_planner/GetPotentialProxemicsLocations.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <math.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv2/ml/ml.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <cppconn/metadata.h>
#include <cppconn/resultset_metadata.h>
#include <cppconn/exception.h>
#include <cppconn/warning.h>

using namespace std;
using namespace sql;

class Proxemics
{
public:

  Proxemics(){};

  ~Proxemics(){};

  void init(ros::NodeHandle nh, string dbhost, string user, string password, string database)
  {
    node_handle_ = nh;

    // stored the database information
    DBHOST = dbhost;
    USER = user;
    PASSWORD = password;
    DATABASE = database;

    map_resolution_ = 0;

    robotRadius = 0.6;          //need to manually change this variable based on the robot
    personRadius = 0.4;

    static_map_sub_ = node_handle_.subscribe<nav_msgs::OccupancyGrid> ("/map", 1, &Proxemics::updateMapCallback, this);

    service_server_get_potential_proxemics_locations_
        = node_handle_.advertiseService("get_potential_proxemics_locations",
                                        &Proxemics::getPotentialProxemicsLocations, this);

    ROS_INFO("Ready to provide potential proxemics based robot target pose.");
  }

  struct ProcTable
  {
    float robotApproachOrientationId;
    float orientation;
    float priority;
  };

  struct Bearing
  {
    float distance;
    float orientation;

    Bearing()
    {
      distance = 0.0;
      orientation = 0.0;
    }


  };

  struct DistanceWithPriority
  {
   float distance;
   int  priority;

   DistanceWithPriority()
   {
     distance = 0.0;
     priority = 0;
    }
  };

  struct OrientationWithPriority
  {
    float orientation;
    int priority;

    OrientationWithPriority()
    {
      orientation = 0.0;
      priority = 0;
    }
  };

  struct BearingWithPriority
  {
    DistanceWithPriority distance;
    OrientationWithPriority orientation;
  };

  struct Pose
  {
    float x; //in [m]
    float y; //in [m]
    float orientation; //in [rad]

    Pose()
    {
      x = 0;
      y = 0;
      orientation = 0;
    }

    Pose(float x_, float y_, float orientation_)
    {
      x = x_;
      y = y_;
      orientation = orientation_;
    }
  };

  template<class T>
    T convertFromMeterToPixelCoordinates(const Pose& pose);

  template<class T>
    string to_string(const T& t);

  float degree2radian(float degree);
  float radian2degree(float radian);

  bool isUserIn(int userId, string locationName);

/*  void getPotentialProxemicsLocations_Kitchen(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
      accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res);
*/
  bool getPotentialProxemicsLocations_Standing(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                               accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res);

  bool getPotentialProxemicsLocations_ExceptionCase(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                              accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res);

  float getUserRadius(float thetaInRadian, float halfShoulderWidth, float halfChestDepth);

  bool getPotentialProxemicsLocations(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
                                      accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res);



  Bearing retrieveProxemicsPreferences(int userId, int robotGenericTaskId);
  BearingWithPriority retrieveProxemicsPreferencesWithPriority(int userId, int robotGenericTaskId);

  void retrieveProxemicsPreferences_ranking(int userId, int robotGenericTaskId,  Bearing *);
  void rankDistanceBasedPriority( int distancePriority, Bearing* );

  Pose calculateRobotPoseFromProxemicsPreference(geometry_msgs::Pose &userPose, Bearing prefBearing);

  Pose getRobotPose(void);

  bool validApproachPosition(Pose personLocation, Pose robotLocation, Pose potentialApproachPose);
  void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);


protected:

  string DBHOST;
  string USER;
  string PASSWORD;
  string DATABASE;

  tf::TransformListener listener;
  tf::StampedTransform tfTransform;

  ros::NodeHandle node_handle_;
  ros::Subscriber static_map_sub_;

  ros::ServiceServer service_server_get_potential_proxemics_locations_; // Service server providing proxemics locations

  double robotRadius; // in [m]
  double personRadius; // in [m]

  double map_resolution_; // in [m/cell]

  bool SQL_error;

  cv::Mat map_;
  cv::Mat expanded_map_;
  cv::Point2d map_origin_; // in [m]

};

#endif
