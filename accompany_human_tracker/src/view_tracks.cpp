
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_human_tracker/TrackedHuman.h>
#include <accompany_human_tracker/TrackedHumans.h>
#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <limits>
#include <iostream>
using namespace std;

//globals
IplImage* img;
int imgWidth=640;
int imgHeight=480;
int waitTime = 30;

class Viewport
{
public:
  
  Viewport()
  {
    reset();
  }
  
  void reset()
  {
    minX=numeric_limits<float>::max();
    maxX=-numeric_limits<float>::max();
    minY=numeric_limits<float>::max();
    maxY=-numeric_limits<float>::max();
    scale=0;
    midX=0;
    midY=0;
  }

  void updateMinMax(float x,float y)
  {
    if (x<minX) minX=x;
    if (x>maxX) maxX=x;
    if (y<minY) minY=y;
    if (y>maxY) maxY=y;
  }

  void finalize()
  {
    if (maxX-minX>0.01 && maxY-minY>0.01)
    {
      float sx=imgWidth/(maxX-minX);
      float sy=imgHeight/(maxY-minY);
      if (sx<sy)
        scale=sx*0.9;
      else
        scale=sy*0.9;
    }
    else
      scale=1;
    midX=(maxX+minX)/2;
    midY=(maxY+minY)/2;
  }

  float minX,maxX,minY,maxY;
  float midX,midY,scale;
};

class Viewports
{
public:

  Viewports()
  {
    currentViewport=0;
    
  }

  void next()
  {
    viewports[currentViewport].reset();
    currentViewport=(currentViewport+1)%2;
    viewports[currentViewport].finalize();
  }

  float scaledLength(float l)
  {
    return l*viewports[currentViewport].scale;
  }

  CvPoint scaledCvPoint(float x,float y)
  {
    float sx=imgWidth/2+(x-viewports[currentViewport].midX)*viewports[currentViewport].scale;
    float sy=imgHeight/2+(viewports[currentViewport].midY-y)*viewports[currentViewport].scale;
    CvPoint point=cvPoint(sx,sy);
    viewports[(currentViewport+1)%2].updateMinMax(x,y);
    return point;
  }

private:
  int currentViewport;
  Viewport viewports[2];
};

Viewports viewports;

// drawing
float viewMin=0;
float viewMax=7;
CvFont font;

tf::TransformListener *listener=NULL;

void trackedHumansReceived(const accompany_human_tracker::TrackedHumans::ConstPtr& trackedHumans)
{
  cvShowImage("view_tracks",img);
  cvWaitKey(waitTime);
  cvSet(img, cvScalar(0,0,0));
  viewports.next();

  cvCircle(img, viewports.scaledCvPoint(viewMin,viewMin), viewports.scaledLength(0.1), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMin,viewMax), viewports.scaledLength(0.2), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMax,viewMin), viewports.scaledLength(0.3), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMax,viewMax), viewports.scaledLength(0.4), cvScalar(255,0,0), 1);
  
  for (unsigned int i=0;i<trackedHumans->trackedHumans.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::Vector3Stamped transVec;
      listener->transformVector("/map",
                                trackedHumans->trackedHumans[i].location,
                                transVec);
      float x=transVec.vector.x;
      float y=transVec.vector.y;
      int id=trackedHumans->trackedHumans[i].id;
      string identity=trackedHumans->trackedHumans[i].identity;
      stringstream ss;ss<<id;
      if (identity.length()>0)
        ss<<","<<identity;
      string name=ss.str();
      cvCircle(img, viewports.scaledCvPoint(x,y), viewports.scaledLength(0.1), cvScalar(0,255,0), 1);
      cvPutText(img,name.c_str(),viewports.scaledCvPoint(x,y),&font,cvScalar(0,255,0));
    }
    catch (tf::TransformException e)
    {
      cerr<<"failed to transform trackedHumans: "<<e.what()<<endl;
    }
  }
}

void humanLocationsReceived(const accompany_human_tracker::HumanLocations::ConstPtr& humanLocations)
{
  for (unsigned int i=0;i<humanLocations->locations.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::Vector3Stamped transVec;
      listener->transformVector("/map",
                                humanLocations->locations[i],
                                transVec);
      float x=transVec.vector.x;
      float y=transVec.vector.y;
      cvCircle(img, viewports.scaledCvPoint(x,y), viewports.scaledLength(0.05), cvScalar(255,255,0),1);
    }
    catch (tf::TransformException e)
    {
      cerr<<"failed to transform humanLocations: "<<e.what()<<endl;
    }
  }
}

void identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& identifiedHumans)
{
  for (unsigned int i=0;i<identifiedHumans->detections.size();i++)
  {
    const geometry_msgs::PoseStamped pose=identifiedHumans->detections[i].pose;    
    try// transform to map coordinate system
    {
      string identity=identifiedHumans->detections[i].label;
      geometry_msgs::PoseStamped transPose;
      listener->transformPose("/map",
                              pose,
                              transPose);
      float x=transPose.pose.position.x;
      float y=transPose.pose.position.y;
      cvCircle(img, viewports.scaledCvPoint(x,y), viewports.scaledLength(0.2), cvScalar(0,0,255),2);
      cvPutText(img,identity.c_str(),viewports.scaledCvPoint(x,y),&font,cvScalar(0,0,255));
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human identity: "<<e.what()<<endl;
    }
  }
}



int main(int argc,char **argv)
{
  ros::init(argc, argv, "view_tracks");
  cout<<"view_tracks"<<endl;

  img=cvCreateImage(cvSize(imgWidth,imgHeight),IPL_DEPTH_8U,3);
  cvSetZero(img);
  cvShowImage("view_tracks",img);
  cvWaitKey(waitTime);
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1,1);

  // create publisher and subscribers
  ros::NodeHandle n;
  tf::TransformListener initListener;
  listener=&initListener;
  ros::Subscriber trackedHumansSub=n.subscribe<accompany_human_tracker::TrackedHumans>("/trackedHumans",10,trackedHumansReceived);
  ros::Subscriber humanLocationsSub=n.subscribe<accompany_human_tracker::HumanLocations>("/humanLocations",10,humanLocationsReceived);
  ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10,identityReceived);
  ros::spin();
  cvReleaseImage(&img);

  return 0;
}
