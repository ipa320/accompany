
#include <Helpers.hh>
#include <CamCalib.hh>
#include <ImageMask.h>

#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace boost::program_options;
using namespace std;

string imageWindow="occlusion by background";
IplImage *image=NULL;
IplImage *imageDraw=NULL;
int camIndex=-1;
ImageMask imageMask;

void printManual()
{
  cout<<"-------- manual ----------"<<endl;
  cout<<"first draw green region to indicate the region of the prior that is occluded by static background"<<endl;
  cout<<"then draw blue region to indicate the static background that occludes"<<endl;
  cout<<"draw using:"<<endl;
  cout<<"  left-click   = add point to current region"<<endl;
  cout<<"  right-click  = end region"<<endl;
  cout<<"  middle-click = undo last operation"<<endl;
  cout<<"--------------------------"<<endl<<endl;
}

void updateMask()
{
  cout<<imageMask<<endl;
  cvCopy(image,imageDraw);
  imageMask.plot(imageDraw);
  cvShowImage(imageWindow.c_str(),imageDraw);
  printManual();
}

void mouseHandler(int event, int x, int y, int flags, void *param)
{
  bool update=false;
  switch (event)
  {
  case CV_EVENT_LBUTTONUP:
    imageMask.add(cvPoint(x,y));
    update=true;
    break;
  case  CV_EVENT_RBUTTONDOWN:
    imageMask.end();
    update=true;
    break;
  case CV_EVENT_MBUTTONDOWN:
    imageMask.undo();
    update=true;
    break;
  }
  if (update)
    updateMask();
}

void plotHull(IplImage *img, vector<WorldPoint> &hull, unsigned c)
{
  hull.push_back(hull.front());
  for (unsigned i = 1; i < hull.size(); ++i)
    cvLine(img, cam[c].project(hull[i - 1]), cam[c].project(hull[i]),
        CV_RGB(255,0,0), 2);
}

int main(int argc, char **argv) 
{
  string imageName,paramsPath,cameraName,maskName;

  // handling arguments
  options_description optionsDescription("Select occlusion by background mask to indicate where the static background occludes the eara of interest\n");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("image-name,i", value<string>(&imageName)->required(),"filename of image to select occlusion by background mask in")
    ("params-path,p", value<string>(&paramsPath)->required(),"the path to the params.xml")
    ("camera-name,c", value<string>(&cameraName)->required(),"the camera name which took the image")
    ("mask-name,m", value<string>(&maskName)->default_value("occlusionBGMask.txt"),"the name of the mask")
    ;

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

  // load data from file
  string params_file = paramsPath + "/" + "params.xml";
  string prior_file = paramsPath + "/" + "prior.txt";
  loadCalibrations(params_file.c_str(),paramsPath.c_str());
  vector<WorldPoint> priorHull;
  loadWorldPriorHull(prior_file.c_str(),priorHull);

  // search for camera
  cout<<"known cameras:"<<endl;
  for (unsigned i=0;i<cam.size();i++)
  {
    cout<<"  "<<cam[i].name;
    if (cam[i].name.compare(cameraName)==0)
    {
      camIndex=i;
      cout<<"  <====== using";
    }
    cout<<endl;
  }
  if (camIndex<0)
  {
    cout<<"camera name '"<<cameraName<<"' not found, use '-c' to select the camera that toke the image"<<endl;
    return 1;
  }

  if (cam[camIndex].occlusionBGMaskFile.compare("")!=0 /*&& !variablesMap.count("mask-name")*/)
  {
    maskName=cam[camIndex].occlusionBGMaskFile;
    cout<<"use existing mask name'"<<maskName<<"'."<<endl;
  }

  // load existing mask
  string mask_file = paramsPath + "/" + maskName;
  ifstream maskin(mask_file.c_str());
  if (maskin.is_open())
  {
    cout<<"read existing mask '"<<mask_file<<"'."<<endl;
    maskin>>imageMask;
    maskin.close();
    cout<<imageMask<<endl;
  }
  else
    cout<<"no existing mask '"<<mask_file<<"' found, start new mask"<<endl;

  
  // load image
  image=loadImage(imageName.c_str());
  plotHull(image,priorHull,camIndex);
  imageDraw=cvCloneImage(image);
  cvNamedWindow(imageWindow.c_str());
  cvSetMouseCallback(imageWindow.c_str(),mouseHandler,NULL);

  updateMask();

  int key = 0;
  while ((char)key != 'q') {
    // cvShowImage("image", src);
    key = cvWaitKey(32);
  }
  
  ofstream maskout(mask_file.c_str());
  if (maskout.is_open())
  {
    cout<<"writing mask to '"<<mask_file<<"'"<<endl;
    maskout<<imageMask;
    maskout.close();
  }
  else
    cout<<"fails to save mask to '"<<mask_file<<"'"<<endl;

  return 0;
}
