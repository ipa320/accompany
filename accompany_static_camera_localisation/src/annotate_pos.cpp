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
#include <cmn/GnuPlot.hh>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

// meaningless comment.
// and another one.

// unsigned width, height;
// int depth=0, channels=0;
// unsigned halfresX = 0, halfresY = 0;
CvScalar BLUE = CV_RGB(0,0,255), RED = CV_RGB(255,0,0), CLR = CV_RGB(0,255,0);
double scale = 1; // TODO

vector<IplImage *> img;
vector< WorldPoint > priorHull;

// vector<scanline_t> merged, prior;
vector<WorldPoint> locations;
const char *win[] = { "1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20"};

CvPoint operator*(double d, const CvPoint &p) {
  return cvPoint(d*p.x,d*p.y);
}

void showScaledImg(unsigned c, IplImage *img)
{
  IplImage *scaled = cvCreateImage(cvSize(img->width*scale,img->height*scale), IPL_DEPTH_8U,3);
  cvResize(img,scaled,CV_INTER_LINEAR);
  plotHull(scaled,priorHull,c,CLR);
  cvShowImage(win[c], scaled);
  cvReleaseImage(&scaled);
}

void refresh()
{
  for (unsigned i=0; i!=img.size(); ++i) {
    IplImage *tmp = cvCloneImage(img[i]);

    vector<CvPoint> tplt;
    for (unsigned j=0;j!=locations.size(); ++j) {
      cam[i].genTemplate(locations[j],tplt);
      plotTemplate(tmp,tplt,BLUE);
    }
    showScaledImg(i,tmp);
    cvReleaseImage(&tmp);
  }
}     

void histogram(const vector<CvPoint> &tplt, IplImage *img, vnl_vector<double> &hist, unsigned iPerBin = 1)
{
  vector<scanline_t> mask;
  getMask(tplt,mask);
  if (hist.size() == 0) {
    hist.set_size(256/iPerBin);
    hist.fill(0.0);
  }
  unsigned char *src = (unsigned char *)img->imageData;
  unsigned K = img->nChannels, denom = K*iPerBin;

  for (unsigned i=0; i<mask.size(); ++i) {
    unsigned char *s=src+mask[i].line*img->widthStep + K*mask[i].start;
    for (unsigned j=mask[i].start; j<mask[i].end; ++j) {
      unsigned val=0.;
      for (unsigned k=0; k!=K; ++k, ++s) {
        val += *s;
        *s = 0;
      }
      val /= denom;
      hist[val]++;
    }
  }
}

std::ostream &operator<<(std::ostream &os, const CvPoint &wp)
{
  return os << "{" << wp.x << "," << wp.y << "}";
}


void mouseHandler(int idx, int event, int x, int y, int flags, void *param)
{
  x /= scale;
  y /= scale;
  WorldPoint pt = cam[idx].getGroundPos(cvPoint(x,y));
  cout << "3D points" << pt.x << "," << pt.y << "," << pt.z << endl;

  static bool down=false;

  vnl_vector<double> h(0);
  switch (event) {
    case CV_EVENT_LBUTTONDOWN:
      cout << "Left button down at " << x << "," << y << endl;
      down = true;
      for (unsigned i=0; i!=img.size(); ++i) {
        IplImage
        *tmp = cvCloneImage(img[i]);
        cvCircle(tmp,cam[i].project(pt),2,CV_RGB(0,255,0),1);

        vector<CvPoint> tplt;
        cam[i].genTemplate(pt,tplt);
        cout << "Camera " << i << endl;
        for (unsigned j=0; j!=tplt.size(); ++j)
	{
	  cout << "camera" << endl;
          cout << tplt[j] << endl;
	}        
	plotTemplate(tmp,tplt,RED);
        // histogram(tplt, tmp, h);
        // plotHull(tmp,tplt);
        showScaledImg(i,tmp);
        cvReleaseImage(&tmp);
      }
//      gp << "plot " << GPBinary(h) << " with boxes\n";
      break;
    case CV_EVENT_MOUSEMOVE:
      if (down)
        for (unsigned i=0; i!=img.size(); ++i) {
          IplImage
          *tmp = cvCloneImage(img[i]);
          cvCircle(tmp,cam[i].project(pt),2,CV_RGB(0,255,0),1);

          vector<CvPoint> tplt;
          cam[i].genTemplate(pt,tplt);
          plotTemplate(tmp,tplt,RED);
          // histogram(tplt, tmp, h);
          // plotHull(tmp,tplt);

          showScaledImg(i,tmp);
          cvReleaseImage(&tmp);
//          gp << "plot " << GPBinary(h) << " with boxes\n";
        }
      break;
    case CV_EVENT_LBUTTONUP:
      down = false;
      locations.push_back(pt);
      cout << "Up at (" << x << "," << y << ")" << endl;
      for (unsigned i=0; i!=img.size(); ++i) {
        IplImage
        *tmp = cvCloneImage(img[i]);
        // cvCircle(tmp,cam[i].project(pt),2,CV_RGB(0,255,0),1);

        vector<CvPoint> tplt;
        for (unsigned j=0;j!=locations.size(); ++j) {
          cam[i].genTemplate(locations[j],tplt);
          plotTemplate(tmp,tplt,BLUE);
          plotHull(tmp,tplt);
        }
        showScaledImg(i,tmp);
        cvReleaseImage(&tmp);
      }

      break;
  }
}

#define DEF(IDX) void mh##IDX(int e, int x, int y, int f, void *p) { return mouseHandler(IDX,e,x,y,f,p); }
DEF(0) DEF(1) DEF(2) DEF(3) DEF(4) DEF(5) DEF(6) DEF(7) DEF(8) DEF(9) DEF(10)
																																																																																																																																																																								    DEF(11) DEF(12) DEF(13) DEF(14) DEF(15) DEF(16) DEF(17)DEF(18) DEF(19) DEF(20)

																																																																																																																																																																								    typedef void (*mh_t)(int,int,int,int,void*);
mh_t mh[] = { mh0,mh1,mh2,mh3,mh4,mh5,mh6,mh7,mh8,mh9,mh10,mh11,mh12,mh13,mh14,mh15,mh16,mh17,mh18,mh19,mh20 };




int main(int argc, char **argv)
{
  string imagelist_file, params_file, prior_file, intrinsic_file, extrinsic_file,annotated_file;

  // handling arguments
  po::options_description optionsDescription("Select prior locations where people can walk\nAllowed options\n");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("list_of_image,l", po::value<string>(&imagelist_file)->required(),"the input image list showing the ground plane\n")
    ("params,p", po::value<string>(&params_file)->required(),"the input xml file containing all parameters\n")
    ("prior,r", po::value<string>(&prior_file)->required(),"the prior\n")
    ("intrinsic,i", po::value<string>(&intrinsic_file)->required(),"camera intrinsic parameter \"intrinsic.xml\"\n")
    ("extrinsic,e", po::value<string>(&extrinsic_file)->required(),"camera extrinsic parameter \"extrinsic.xml\"\n")
    ("annotated,a", po::value<string>(&annotated_file)->required(),"file to store annotated positions\n")
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

  vector<vector<string> > imgs;
  listImages(imagelist_file.c_str(), imgs);
  cam = vector<CamCalib>(imgs[0].size());
  img = vector<IplImage*>(imgs[0].size());

  unsigned index = 0;

  for (unsigned i = 0; i != imgs[index].size(); ++i)
  {
    cout << "Loading " << imgs[index][i] << endl;
    img[i] = loadImage(imgs[index][i].c_str());

    cvNamedWindow(win[i]);
    cvSetMouseCallback(win[i], mh[i], NULL);
    showScaledImg(i, img[i]);
  }
  width = img[0]->width;
  height = img[0]->height;
  depth = img[0]->depth;
  channels = img[0]->nChannels;
  halfresX = width / 2;
  halfresY = height / 2;

  loadCalibrations(params_file.c_str());
  index++;

  loadHull(prior_file.c_str(), priorHull);

  ofstream ofs(annotated_file.c_str());
  if (!ofs)
    errx(1, "Cannot open file %s", argv[4]);

  int key = 0;
  while ((char) key != 'q' && index < imgs.size())
  {
    key = cvWaitKey(0);
    switch ((char) key)
    {
      case 10:
        ofs << imgs[index][0].substr(imgs[index][0].rfind("/") + 1) << " ";
        for (unsigned i = 0; i != locations.size(); ++i)
          ofs << "(" << locations[i].x << "," << locations[i].y << ") ";
        ofs << endl;

        locations.clear();
        for (unsigned i = 0; i != imgs[index].size(); ++i)
        {
          cvReleaseImage(&img[i]);
          img[i] = loadImage(imgs[index][i].c_str());
          showScaledImg(i, img[i]);
        }
        index++;
        if (index == imgs.size())
          goto stop;
        break;
      case '-':
        if (scale > .1)
          scale -= .1;
        refresh();
        break;
      case '+':
        if (scale < 2)
          scale += .1;
        refresh();
        break;
      case 'z':
        if (locations.size())
          locations.pop_back();
        refresh();
        break;
      default:
        cout << "c = " << key << endl;
        break;
    }
  }

  stop: for (unsigned i = 0; i != img.size(); ++i)
    cvReleaseImage(&img[i]);
  return 0;
}
