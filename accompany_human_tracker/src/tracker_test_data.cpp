
#include <ros/ros.h>

#include <accompany_human_tracker/HumanLocations.h>
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
  if (range<=0)
    return 0;
  else
    return rand()%range+min;
}

void move(accompany_human_tracker::HumanLocations &humanLocations,vector<geometry_msgs::Vector3> &speeds)
{
  vector<geometry_msgs::Vector3>::iterator lit=humanLocations.locations.begin();
  vector<geometry_msgs::Vector3>::iterator sit=speeds.begin();
  while (lit!=humanLocations.locations.end() || sit!=speeds.end())
  {
    // move
    lit->x+=sit->x;
    lit->y+=sit->y;
    lit->z+=sit->z;
    
    // check borders
    if (lit->x<MIN_X || lit->x>MAX_X)
      sit->x*=-1;
    if (lit->y<MIN_Y || lit->y>MAX_Y)
      sit->y*=-1;
    if (lit->z<MIN_Z || lit->z>MAX_Z)
      sit->z*=-1;

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
  ros::Publisher humanLocationPub=n.advertise<accompany_human_tracker::HumanLocations>("/humanLocations",10);
  ros::Publisher humanIdentityPub=n.advertise<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10);
  
  // init humans
  int nrHumans=4;
  accompany_human_tracker::HumanLocations humanLocations;
  geometry_msgs::Vector3 location;
  vector<geometry_msgs::Vector3> speeds;
  geometry_msgs::Vector3 speed;
  for (int i=0;i<nrHumans;i++)
  {
    location.x=rand(0,MAX_X);
    location.y=rand(0,MAX_Y);
    location.z=rand(0,MAX_Z);
    humanLocations.locations.push_back(location);
    speed.x=rand(-MAX_X,MAX_X)/100;
    speed.y=rand(-MAX_Y,MAX_Y)/100;
    speed.z=rand(-MAX_Z,MAX_Z)/100;
    speeds.push_back(speed);
  }
  
  // init identities
  cob_people_detection_msgs::DetectionArray detectionArray;
  cob_people_detection_msgs::Detection detection;
  detection.label="person1";
  detection.pose.pose.position.x=1;
  detection.pose.pose.position.y=1;
  detection.pose.pose.position.z=0;
  detection.pose.pose.orientation.x=0;
  detection.pose.pose.orientation.y=0;
  detection.pose.pose.orientation.z=0;
  detection.pose.pose.orientation.w=1;
  detection.pose.header.frame_id="head_cam3d_link";
  detectionArray.detections.push_back(detection);
  detection.label="person2";
  detection.pose.pose.position.x=5;
  detection.pose.pose.position.y=5;
  detection.pose.pose.position.z=0;
  detection.pose.pose.orientation.x=0;
  detection.pose.pose.orientation.y=0;
  detection.pose.pose.orientation.z=0;
  detection.pose.pose.orientation.w=1;
  detection.pose.header.frame_id="head_cam3d_link";
  detectionArray.detections.push_back(detection);
  
  ros::Rate loop_rate(8);
  while (ros::ok())
  {
    move(humanLocations,speeds);
    humanLocationPub.publish(humanLocations);

    vector<cob_people_detection_msgs::Detection>::iterator it=detectionArray.detections.begin();
    it->pose.header.stamp=ros::Time::now();
    humanIdentityPub.publish(detectionArray);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

