
#include <ros/ros.h>

#include <accompany_uva_msg/HumanLocations.h>
#include <cob_people_detection_msgs/DetectionArray.h>

#include <vector>
using namespace std;

#define MIN_X 0
#define MIN_Y 0
#define MIN_Z 0

#define MAX_X 6
#define MAX_Y 6
#define MAX_Z 0

double rand(int min,int max)
{
  int range=max-min;
  return ((rand()/(double)RAND_MAX)*range)+min;
}

void move(accompany_uva_msg::HumanLocations &humanLocations,vector<geometry_msgs::PointStamped> &speeds)
{
  vector<geometry_msgs::PointStamped>::iterator lit=humanLocations.locations.begin();
  vector<geometry_msgs::PointStamped>::iterator sit=speeds.begin();
  while (lit!=humanLocations.locations.end() || sit!=speeds.end())
  {
    // move
    lit->point.x+=sit->point.x;
    lit->point.y+=sit->point.y;
    lit->point.z+=sit->point.z;
    
    // check borders
    if (lit->point.x<MIN_X || lit->point.x>MAX_X)
      sit->point.x*=-1;
    if (lit->point.y<MIN_Y || lit->point.y>MAX_Y)
      sit->point.y*=-1;
    if (lit->point.z<MIN_Z || lit->point.z>MAX_Z)
      sit->point.z*=-1;

    // next
    lit++;
    sit++;
  }
}

void move(cob_people_detection_msgs::DetectionArray &detectionArray,vector<geometry_msgs::PointStamped> &speeds)
{
  vector<cob_people_detection_msgs::Detection>::iterator lit=detectionArray.detections.begin();
  vector<geometry_msgs::PointStamped>::iterator sit=speeds.begin();
  while (lit!=detectionArray.detections.end() || sit!=speeds.end())
  {
    // move
    lit->pose.pose.position.x+=sit->point.x;
    lit->pose.pose.position.y+=sit->point.y;
    lit->pose.pose.position.z+=sit->point.z;

    // check borders
    if (lit->pose.pose.position.x<MIN_X || lit->pose.pose.position.x>MAX_X)
      sit->point.x*=-1;
    if (lit->pose.pose.position.y<MIN_Y || lit->pose.pose.position.y>MAX_Y)
      sit->point.y*=-1;
    if (lit->pose.pose.position.z<MIN_Z || lit->pose.pose.position.z>MAX_Z)
      sit->point.z*=-1;

    // next
    lit++;
    sit++;
  }
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "test_data");

  // create publisher and subscribers
  ros::NodeHandle n;
  ros::Publisher humanLocationPub=n.advertise<accompany_uva_msg::HumanLocations>("/humanLocations",10);
  ros::Publisher humanIdentityPub=n.advertise<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10);
  
  // init humans
  int nrHumans1=4;
  accompany_uva_msg::HumanLocations humanLocations1;
  vector<geometry_msgs::PointStamped> humanSpeeds1;
  for (int i=0;i<nrHumans1;i++)
  {
    geometry_msgs::PointStamped location;
    location.header.frame_id="/overhead1";
    location.point.x=rand(0,MAX_X);
    location.point.y=rand(0,MAX_Y);
    location.point.z=rand(0,MAX_Z);
    humanLocations1.locations.push_back(location);
    geometry_msgs::PointStamped speed;
    speed.point.x=rand(-MAX_X,MAX_X)/100;
    speed.point.y=rand(-MAX_Y,MAX_Y)/100;
    speed.point.z=rand(-MAX_Z,MAX_Z)/100;
    humanSpeeds1.push_back(speed);
  }

  // init humans
  int nrHumans2=4;
  accompany_uva_msg::HumanLocations humanLocations2;
  vector<geometry_msgs::PointStamped> humanSpeeds2;
  for (int i=0;i<nrHumans2;i++)
  {
    geometry_msgs::PointStamped location;
    location.header.frame_id="/overhead2";
    location.point.x=rand(0,MAX_X);
    location.point.y=rand(0,MAX_Y);
    location.point.z=rand(0,MAX_Z);
    humanLocations2.locations.push_back(location);
    geometry_msgs::PointStamped speed;
    speed.point.x=rand(-MAX_X,MAX_X)/100;
    speed.point.y=rand(-MAX_Y,MAX_Y)/100;
    speed.point.z=rand(-MAX_Z,MAX_Z)/100;
    humanSpeeds2.push_back(speed);
  }
  
  // init identities
  int nrIdentities=3;
  cob_people_detection_msgs::DetectionArray detectionArray;
  vector<geometry_msgs::PointStamped> identitySpeeds;
  for (int i=0;i<nrIdentities;i++)
  {
    cob_people_detection_msgs::Detection detection;
    stringstream ss;
    ss<<"person";
    ss<<(char)(65+i);
    detection.label=ss.str();
    detection.pose.header.frame_id="/head_cam3d_link";
    detection.pose.pose.position.x=rand(0,MAX_X);
    detection.pose.pose.position.y=rand(0,MAX_Y);
    detection.pose.pose.position.z=rand(0,MAX_Z);
    detection.pose.pose.orientation.x=0;
    detection.pose.pose.orientation.y=0;
    detection.pose.pose.orientation.z=0;
    detection.pose.pose.orientation.w=1;
    detectionArray.detections.push_back(detection);
    geometry_msgs::PointStamped speed;
    speed.point.x=rand(-MAX_X,MAX_X)/100;
    speed.point.y=rand(-MAX_Y,MAX_Y)/100;
    speed.point.z=rand(-MAX_Z,MAX_Z)/100;
    identitySpeeds.push_back(speed);
  }
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    move(humanLocations1,humanSpeeds1);
    humanLocationPub.publish(humanLocations1);

    move(humanLocations2,humanSpeeds2);
    humanLocationPub.publish(humanLocations2);

    move(detectionArray,identitySpeeds);
    cob_people_detection_msgs::DetectionArray sendDetectionArray;
    for (vector<cob_people_detection_msgs::Detection>::iterator it=detectionArray.detections.begin();
         it!=detectionArray.detections.end();
         it++)
    {
      if (rand()%100<10)
      {
        it->pose.header.stamp=ros::Time::now();
        sendDetectionArray.detections.push_back(*it);
      }
    }
    if (sendDetectionArray.detections.size()>0)
      humanIdentityPub.publish(sendDetectionArray);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

