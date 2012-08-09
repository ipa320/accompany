
#include <ros/ros.h>
#include <accompany_human_tracker/TrackedHuman.h>
#include <accompany_human_tracker/TrackedHumans.h>
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

// scaling
double minX;
double maxX;
double minY;
double maxY;
double scale;
double midX;
double midY;

// drawing
double viewMin=0;
double viewMax=7;
CvFont font;


void initView()
{
  minX=numeric_limits<double>::max();
  maxX=-numeric_limits<double>::max();
  minY=numeric_limits<double>::max();
  maxY=-numeric_limits<double>::max();
  scale=0;
  midX=0;
  midY=0;
}

void updateMinMax(double x,double y)
{
  if (x<minX) minX=x;
  if (x>maxX) maxX=x;
  if (y<minY) minY=y;
  if (y>maxY) maxY=y;
  
  if (maxX-minX>0.01 && maxY-minY>0.01)
  {
    double sx=imgWidth/(maxX-minX);
    double sy=imgHeight/(maxY-minY);
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

double scaledLength(double l)
{
  return l*scale;
}

CvPoint scaledCvPoint(double x,double y)
{
  double sx=imgWidth/2+(x-midX)*scale;
  double sy=imgHeight/2+(midY-y)*scale;
  CvPoint point=cvPoint(sx,sy);
  return point;
}

void trackedHumansReceived(const accompany_human_tracker::TrackedHumans::ConstPtr& trackedHumans)
{
  cvSetZero(img);
  initView();
  updateMinMax(viewMin,viewMin);
  updateMinMax(viewMax,viewMax);

  for (unsigned int i=0;i<trackedHumans->trackedHumans.size();i++)
  {
    double x=trackedHumans->trackedHumans[i].location.x;
    double y=trackedHumans->trackedHumans[i].location.y;
    updateMinMax(x,y);
  }

  cvCircle(img, scaledCvPoint(viewMin,viewMin), scaledLength(0.1), cvScalar(255,0,0), 1);
  cvCircle(img, scaledCvPoint(viewMin,viewMax), scaledLength(0.2), cvScalar(255,0,0), 1);
  cvCircle(img, scaledCvPoint(viewMax,viewMin), scaledLength(0.3), cvScalar(255,0,0), 1);
  cvCircle(img, scaledCvPoint(viewMax,viewMax), scaledLength(0.4), cvScalar(255,0,0), 1);
  
  for (unsigned int i=0;i<trackedHumans->trackedHumans.size();i++)
  {
    double x=trackedHumans->trackedHumans[i].location.x;
    double y=trackedHumans->trackedHumans[i].location.y;
    int id=trackedHumans->trackedHumans[i].id;
    stringstream ss;ss<<id;
    string name=ss.str();
    cvCircle(img, scaledCvPoint(x,y), scaledLength(0.1), cvScalar(0,255,0), 1);
    cvPutText(img,name.c_str(),scaledCvPoint(x,y),&font,cvScalar(0,255,0));
  }

  cvShowImage("view_tracks",img);
  cvWaitKey(waitTime);
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
  ros::Subscriber trackedHumansSub=n.subscribe<accompany_human_tracker::TrackedHumans>("/trackedHumans",10,trackedHumansReceived);
  ros::spin();
  cvReleaseImage(&img);

  return 0;
}
