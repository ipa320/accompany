
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <accompany_uva_msg/HumanLocations.h>
#include <accompany_uva_msg/TrackedHuman.h>
#include <accompany_uva_msg/TrackedHumans.h>
#include <cob_people_detection_msgs/Detection.h>
#include <cob_people_detection_msgs/DetectionArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

#include <limits>
#include <iostream>
using namespace std;
using namespace boost;

//globals
IplImage* img;
int imgWidth=380;
int imgHeight=380;
int waitTime=30;

// params
string saveImagesPath;
string imagePostfix;
string mapFile;
IplImage* mapImage=NULL;

#define nrColors 5
CvScalar colors[nrColors];

vector<cob_people_detection_msgs::DetectionArray> identifiedHumansStore;
map<string,accompany_uva_msg::HumanLocations> humanLocationStore;

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

  CvPoint scaledCvPoint(float x,float y,int update=0)
  {
    float sx=imgWidth/2+(x-viewports[currentViewport].midX)*viewports[currentViewport].scale;
    float sy=imgHeight/2+(viewports[currentViewport].midY-y)*viewports[currentViewport].scale;
    CvPoint point=cvPoint(sx,sy);
    if (update)
      viewports[(currentViewport+1)%2].updateMinMax(x,y);
    return point;
  }

private:
  int currentViewport;
  Viewport viewports[2];
};

void myCvPutText(CvArr* img, const char* text, CvPoint org, const CvFont* font, CvScalar color)
{
  cvPutText(img,text,org,font,color);
  cvPutText(img,text,cvPoint(org.x+1,org.y),font,color);
  cvPutText(img,text,cvPoint(org.x,org.y+1),font,color);
  cvPutText(img,text,cvPoint(org.x+1,org.y+1),font,color);
}

Viewports viewports;

// drawing
float viewMin=-2;
float viewMax=8;
CvFont font;

tf::TransformListener *listener=NULL;

void drawTrackedHumans(const accompany_uva_msg::TrackedHumans::ConstPtr& trackedHumans)
{
  for (unsigned int i=0;i<trackedHumans->trackedHumans.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::PointStamped transVec;
      listener->transformPoint("/map",
                                trackedHumans->trackedHumans[i].location,
                                transVec);
      float x=transVec.point.x;
      float y=transVec.point.y;
      int id=trackedHumans->trackedHumans[i].id;
      string identity=trackedHumans->trackedHumans[i].identity;
      stringstream ss;ss<<id;
      if (identity.length()>0)
        ss<<","<<identity;
      string name=ss.str();
      cvCircle(img, viewports.scaledCvPoint(x,y,1), viewports.scaledLength(0.25), cvScalar(0,0,255),2);
      myCvPutText(img,name.c_str(),viewports.scaledCvPoint(x,y,1),&font,cvScalar(0,0,255));
    }
    catch (tf::TransformException e)
    {
      cerr<<"failed to transform trackedHumans: "<<e.what()<<endl;
    }
  }
}

void drawIdentifiedHumans(const cob_people_detection_msgs::DetectionArray& identifiedHumans,int color)
{
  for (unsigned int i=0;i<identifiedHumans.detections.size();i++)
  {
    const geometry_msgs::PoseStamped pose=identifiedHumans.detections[i].pose;    
    try// transform to map coordinate system
    {
      string identity=identifiedHumans.detections[i].label;
      geometry_msgs::PoseStamped transPose;
      listener->transformPose("/map",
                              pose,
                              transPose);
      float x=transPose.pose.position.x;
      float y=transPose.pose.position.y;
      cvCircle(img,viewports.scaledCvPoint(x,y),viewports.scaledLength(0.2),colors[color%nrColors],2);
      cvPutText(img,identity.c_str(),viewports.scaledCvPoint(x,y),&font,colors[color%nrColors]);
    }
    catch (tf::TransformException e)
    {
      cerr<<"error while tranforming human identity: "<<e.what()<<endl;
    }
  }
}

void drawHumanLocations(const accompany_uva_msg::HumanLocations& humanLocations,int color)
{
  for (unsigned int i=0;i<humanLocations.locations.size();i++)
  {
    try// transform to map coordinate system
    {
      geometry_msgs::PointStamped transVec;
      listener->transformPoint("/map",
                                humanLocations.locations[i],
                                transVec);
      float x=transVec.point.x;
      float y=transVec.point.y;
      cvCircle(img,viewports.scaledCvPoint(x,y),viewports.scaledLength(0.08),colors[color%nrColors],2);
    }
    catch (tf::TransformException e)
    {
      cerr<<"failed to transform humanLocations: "<<e.what()<<endl;
    }
  }
}

void trackedHumansReceived(const accompany_uva_msg::TrackedHumans::ConstPtr& trackedHumans)
{
  if (mapImage==NULL)
    cvSet(img, cvScalar(0,0,0));
  else
  {
    cvCopy( mapImage, img, NULL );
  }

  cvCircle(img, viewports.scaledCvPoint(viewMin,viewMin,1), viewports.scaledLength(0.1), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMin,viewMax,1), viewports.scaledLength(0.1), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMax,viewMin,1), viewports.scaledLength(0.1), cvScalar(255,0,0), 1);
  cvCircle(img, viewports.scaledCvPoint(viewMax,viewMax,1), viewports.scaledLength(0.1), cvScalar(255,0,0), 1);
  
  
  drawTrackedHumans(trackedHumans); // draw tracked humans
  
  for (unsigned int i=0;i<identifiedHumansStore.size();i++) // draw last identified humans
    drawIdentifiedHumans(identifiedHumansStore[i],i);
  identifiedHumansStore.clear();

  int c=0;
  for (map<string,accompany_uva_msg::HumanLocations>::iterator it=humanLocationStore.begin(); // draw last identified humans
       it!=humanLocationStore.end();it++)
  {
    drawHumanLocations(it->second,c);
    c++;
  } 

  if (saveImagesPath!="")
  {
    ros::Time begin = ros::Time::now();
    stringstream ss;
    ss<<saveImagesPath<<"/"<<setfill('0')<<setw(12)<<begin.sec
      <<setfill('0')<<setw(9)<<begin.nsec<<imagePostfix<<".png";
    cvSaveImage(ss.str().c_str(),img);
  }
  cvShowImage("view_tracks",img);
  cvWaitKey(waitTime);
  viewports.next();
}

void humanLocationsReceived(const accompany_uva_msg::HumanLocations::ConstPtr& humanLocations)
{
  if (humanLocations->locations.size()>0)
  {
    string frame=humanLocations->locations[0].header.frame_id;
    humanLocationStore[frame]=*humanLocations;
  }
}

void identityReceived(const cob_people_detection_msgs::DetectionArray::ConstPtr& identifiedHumans)
{
  identifiedHumansStore.push_back(*identifiedHumans);
}

void initColor()
{
  colors[0]=cvScalar(255,0,0);
  colors[1]=cvScalar(0,255,0);
  colors[2]=cvScalar(255,255,0);
  colors[3]=cvScalar(0,255,255);
  colors[4]=cvScalar(255,0,255);
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "view_tracks");
  cout<<"view_tracks"<<endl;

  // handling arguments
  program_options::options_description optionsDescription(
      "view_track views human detections and tracks humans");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("saveImagePath,s", program_options::value<string>(&saveImagesPath)->default_value(""),"path to save images to\n")
    ("imagePostfix,i", program_options::value<string>(&imagePostfix)->default_value(""),"postfix of image name\n")
    ("mapFile,m", program_options::value<string>(&mapFile)->default_value(""),"map to be used as background\n");

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

  initColor();
  if (mapFile!="")
  {
    img=cvLoadImage(mapFile.c_str());
    mapImage=cvLoadImage(mapFile.c_str());
    cvCopy(mapImage, img, NULL );
    cvShowImage("view_tracks",img);
    cvWaitKey(waitTime);
  }
  else
    img=cvCreateImage(cvSize(imgWidth,imgHeight),IPL_DEPTH_8U,3);
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1,1);

  // create publisher and subscribers
  ros::NodeHandle n;
  tf::TransformListener initListener;
  listener=&initListener;
  ros::Subscriber trackedHumansSub=n.subscribe<accompany_uva_msg::TrackedHumans>("/trackedHumans",10,trackedHumansReceived);
  ros::Subscriber humanLocationsSub=n.subscribe<accompany_uva_msg::HumanLocations>("/humanLocations",10,humanLocationsReceived);
  ros::Subscriber identitySub=n.subscribe<cob_people_detection_msgs::DetectionArray>("/face_recognitions",10,identityReceived);
  ros::spin();
  cvReleaseImage(&img);

  return 0;
}
