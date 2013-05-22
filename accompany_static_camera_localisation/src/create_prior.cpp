#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <err.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "Helpers.hh"
#include "CamCalib.hh"
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
namespace po = boost::program_options;
using namespace std;

CvScalar CLR = CV_RGB(0,255,0);
vector<IplImage *> img;
vector<IplImage *> imgPlot;
vector<WorldPoint> priorHull;

WorldPoint pt;
bool ptValid=false;
const char *win[] = { "1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20"};

void refreshPlot()
{
  for (unsigned i=0; i!=img.size(); ++i)
  {
    if (imgPlot.size()>i)
      cvCopy(img[i],imgPlot[i]);
    else
      imgPlot.push_back(cvCloneImage(img[i]));
  }
}

void plotPriorHull()
{
  for (unsigned i=0; i!=img.size(); ++i) 
  {
    if (ptValid)
      plotHull(imgPlot[i],priorHull,i,CLR,pt);
    else
      plotHull(imgPlot[i],priorHull,i,CLR);
    cvShowImage(win[i],imgPlot[i]);
  }
}

void plot()
{
  refreshPlot();
  plotPriorHull();
}

void mouseHandler(int idx, int event, int x, int y, int flags, void *)
{
  pt = cam[idx].getGroundPos(cvPoint(x,y));
  ptValid=true;

  switch (event) 
  {
  case CV_EVENT_MOUSEMOVE:
    break;
  case CV_EVENT_LBUTTONUP:
    priorHull.push_back(pt);
    break;
  case CV_EVENT_RBUTTONDOWN:
    if (priorHull.size()>0) priorHull.pop_back();
    break;
  }
  plot();
}

#define DEF(IDX) void mh##IDX(int e, int x, int y, int f, void *p) { return mouseHandler(IDX,e,x,y,f,p); }
DEF(0) DEF(1) DEF(2) DEF(3) DEF(4) DEF(5) DEF(6) DEF(7) DEF(8) DEF(9) DEF(10)
		    DEF(11) DEF(12) DEF(13) DEF(14) DEF(15) DEF(16) DEF(17)DEF(18) DEF(19) DEF(20)
		    typedef void (*mh_t)(int,int,int,int,void*);
mh_t mh[] = { mh0,mh1,mh2,mh3,mh4,mh5,mh6,mh7,mh8,mh9,mh10,mh11,mh12,mh13,mh14,mh15,mh16,mh17,mh18,mh19,mh20 };



int main(int argc, char **argv) {

  string imagelist_file, params_file;

  // handling arguments
  po::options_description optionsDescription("Select prior locations where people can walk\nAllowed options\n");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("list_of_image,l", po::value<string>(&imagelist_file)->required(),"the input image list showing the ground plane\n")
    ("params_file,p", po::value<string>(&params_file)->required(),"filename of params.xml")
    ;

  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription), variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    po::notify(variablesMap);
  }
  catch( const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- "<<e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout <<  optionsDescription << std::endl;
    return 1;
  }

  vector< vector<string> > imgs;
  listImages(imagelist_file.c_str(),imgs);
  cam = vector<CamCalib>(imgs[0].size());
  img = vector<IplImage *>(imgs[0].size());

  unsigned index = 0;
  for (unsigned i=0; i!=imgs[index].size(); ++i)
  {
    img[i] = loadImage(imgs[index][i].c_str());

    cvNamedWindow(win[i]);
    cvSetMouseCallback(win[i], mh[i], NULL);
    cvShowImage(win[i], img[i]);
  }
  width = img[0]->width;
  height = img[0]->height;
  depth = img[0]->depth;
  channels = img[0]->nChannels;
  halfresX = width/2;
  halfresY = height/2;

  boost::filesystem::path p(params_file);
  string path = p.parent_path().string().c_str();
  string prior_file = path + "/" + "prior.txt";

  loadHull(prior_file.c_str(),priorHull);
  loadCalibrations(params_file.c_str());
  plot();

  cout<<"================================"<<endl;
  cout<<"left button : add point"<<endl;
  cout<<"right button: remove point"<<endl;
  cout<<"key 'q'     : save and quit"<<endl;
  cout<<"key Ctrl-C  : quit without saving"<<endl;
  cout<<"================================"<<endl;

  int key = 0;
  while ((char)key != 'q') {
    // cvShowImage("image", src);
    key = cvWaitKey(0);
  }

  cout << "1" << endl;
  for (unsigned i=0; i!=priorHull.size(); ++i)
    cout << priorHull[i].x << " " << priorHull[i].y << " " << priorHull[i].z << endl;

  saveHull(prior_file.c_str(),priorHull);

  cout << endl;
  cout << "prior saved to " << prior_file << endl;

  for (unsigned i=0; i!=img.size(); ++i)
  {
    cvReleaseImage(&img[i]);
    cvReleaseImage(&imgPlot[i]);
  }
  return 0;
}
