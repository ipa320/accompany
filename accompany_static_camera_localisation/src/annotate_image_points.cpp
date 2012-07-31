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

using namespace std;

// meaningless comment.
// and another one.

// unsigned width, height;
// int depth=0, channels=0;
// unsigned halfresX = 0, halfresY = 0;
CvScalar BLUE = CV_RGB(0,0,255), GREEN = CV_RGB(0,255,0), RED = CV_RGB(255,0,0),CLR = CV_RGB(0,255,0);
double scale=1.5, radius = 10;

IplImage *img;
CvPoint shift = cvPoint(10,10);
unsigned selected=0;

struct LocNDir {
     int x, y;
     double angle;

     LocNDir(const CvPoint &p) {
          x = p.x;
          y = p.y;
          angle = 0;
     }
     
     operator CvPoint() {
          return cvPoint(x,y);
     }
     
};


vector<LocNDir> locations;

CvPoint operator*(double d, const CvPoint &p) {
     return cvPoint(d*p.x,d*p.y);
}

CvPoint operator+(const CvPoint &p1, const CvPoint &p2) {
     return cvPoint(p1.x+p2.x,p1.y+p2.y);
}
CvPoint operator-(const CvPoint &p1, const CvPoint &p2) {
     return cvPoint(p1.x-p2.x,p1.y-p2.y);
}
std::ostream &operator<<(std::ostream &os, const CvPoint &wp)
{
     return os << "{" << wp.x << "," << wp.y << "}";
}
// std::ostream &operator<<(std::ostream &os, const LocNDir &wp)
// {
//      return os << "(" << wp.x << "," << wp.y << ")v" << wp.angle;
// }

void showScaledImg(const char *win, IplImage *img)
{
     IplImage
          *scaled = cvCreateImage(cvSize(img->width*scale,img->height*scale), IPL_DEPTH_8U,3);
     cvResize(img,scaled,CV_INTER_LINEAR);
     cvShowImage(win, scaled);
     cvReleaseImage(&scaled);
}

void refresh()
{
     IplImage
          *tmp = cvCloneImage(img);

     for (unsigned j=0;j!=locations.size(); ++j) {
          if (j==selected)
               cvCircle(tmp, locations[j], radius, RED, 2);
          else
               cvCircle(tmp, locations[j], radius, GREEN, 1);
               // cvRectangle(tmp, locations[j]-shift, locations[j]+shift, GREEN, 1);
          cvLine(tmp, locations[j], locations[j] + cvPoint(2*radius*cos(locations[j].angle), 2*radius*sin(locations[j].angle)), RED, 1);
          
     }
     
     showScaledImg("image",tmp);
     cvReleaseImage(&tmp);
}     

// void histogram(const vector<CvPoint> &tplt, IplImage *img, vnl_vector<double> &hist, unsigned iPerBin = 1)
// {
//      vector<scanline_t> mask;
//      getMask(tplt,mask);
//      if (hist.size() == 0) {
//           hist.set_size(256/iPerBin);
//           hist.fill(0.0);
//      } 
//      unsigned char *src = (unsigned char *)img->imageData;
//      unsigned K = img->nChannels, denom = K*iPerBin;
     
//      for (unsigned i=0; i<mask.size(); ++i) {
//           unsigned char *s=src+mask[i].line*img->widthStep + K*mask[i].start;
//           for (unsigned j=mask[i].start; j<mask[i].end; ++j) {
//                unsigned val=0.;
//                for (unsigned k=0; k!=K; ++k, ++s) {
//                     val += *s;
//                     *s = 0;
//                }
//                val /= denom;
//                hist[val]++;
//           }
//      }
// }

void mouseHandler(int event, int x, int y, int flags, void *param)
{
     x /= scale;
     y /= scale;
     CvPoint p = cvPoint(x,y);
     static bool down=false, rdown=false;

     vnl_vector<double> h(0);
     switch (event) {
     case CV_EVENT_LBUTTONDOWN:
     {
          cout << "Left button down at " << x << "," << y << endl;
          down = true;
          IplImage
               *tmp = cvCloneImage(img);
          cvCircle(tmp,p,2,BLUE,1);

          cvRectangle(tmp, p-shift, p+shift, BLUE, 1);
          showScaledImg("image",tmp);
          cvReleaseImage(&tmp);
     }
          break;
     case CV_EVENT_RBUTTONDOWN:
     {
          rdown = true;
          cout << "Right button down at " << x << "," << y << endl;
          CvPoint diff = p-locations[selected];
          locations[selected].angle = atan2(diff.y,diff.x);
          // cout << "diff=" << diff << ", Angle is " << locations[selected].angle << endl;
          refresh();
     }
          break;
     case CV_EVENT_MOUSEMOVE: 
          if (down) {
               IplImage
                    *tmp = cvCloneImage(img);
               cvCircle(tmp,p,2,BLUE,1);
               cvRectangle(tmp, p-shift, p+shift, BLUE, 1);
               showScaledImg("image",tmp);
               cvReleaseImage(&tmp);
          }
          if (rdown) {
               CvPoint diff = p-locations[selected];
               locations[selected].angle = atan2(diff.y,diff.x);
               refresh();
          }               
          break;
     case CV_EVENT_LBUTTONUP:
          down = false;
          selected = locations.size();
          locations.push_back(p);
          cout << "Up at (" << x << "," << y << ")" << endl; 
          refresh();

          break;
     case CV_EVENT_RBUTTONUP:
          rdown = false;
          break;
     }
}

// void loadAnnotations(const char *dir, const char *annotation,
//              vector< vnl_vector<double> > &set)
// {
//      ifstream ifs(annotation);
//      if(!ifs)
//           errx(1,"Could not open %s",annotation);

//      char buffer[64*1024];
//      ifs.getline(buffer, sizeof(buffer));
//      while (!ifs.eof()) {
//           vector<char *>
//                s = splitwhite(buffer,true);
//           string imgS = string(dir)+"/"+s[0];
//           loadProcImgs(imgS.c_str());
//           for (unsigned i=1; i<s.size(); ++i) {
//                vector<char *> v = split(s[i],',');
//                v[0][0] = ' ';   // remove '('
//                // cout << "adding area at (" << v[0] << "," << v[1] << ")" <<endl;
//                CvPoint p1 = cvPoint(atoi(v[0]),atoi(v[1])),
//                     jitter1 = cvPoint(3,3),
//                     jitter2 = cvPoint(3,-3),
//                     pt = p1;
//                if (pt.x >= shift.x && pt.y >= shift.y && pt.x < width-shift.x && pt.y < height-shift.y) 
//                     set.push_back(getArea(pt));
//                pt = p1+jitter1;
//                if (pt.x >= shift.x && pt.y >= shift.y && pt.x < width-shift.x && pt.y < height-shift.y) 
//                     set.push_back(getArea(pt));
//                pt = p1-jitter1;
//                if (pt.x >= shift.x && pt.y >= shift.y && pt.x < width-shift.x && pt.y < height-shift.y) 
//                     set.push_back(getArea(pt));
//                pt = p1+jitter2;
//                if (pt.x >= shift.x && pt.y >= shift.y && pt.x < width-shift.x && pt.y < height-shift.y) 
//                     set.push_back(getArea(pt));
//                pt = p1-jitter2;
//                if (pt.x >= shift.x && pt.y >= shift.y && pt.x < width-shift.x && pt.y < height-shift.y) 
//                     set.push_back(getArea(pt));
//           }
//           cvReleaseImage(&img);
          
//           ifs.getline(buffer, sizeof(buffer));
//      }
//      ifs.close();
// }    



int main(int argc, char **argv)
{
     if (argc != 3)
          errx(2, "usage: <filelist> <annotation.txt>");

     vector< vector<string> >
          imgs;
     listImages(argv[1],imgs);

     unsigned
          index = 0;

     cout << "Loading " << imgs[index][0] << endl;
     img = loadImage(imgs[index][0].c_str());
     if (!img)
          errx(1,"Can't open %s", imgs[index][0].c_str());
          
     cvNamedWindow("image");
     cvSetMouseCallback("image", mouseHandler, NULL);
     showScaledImg("image", img);          
     
     width = img->width;
     height = img->height;
     depth = img->depth;
     channels = img->nChannels;
     halfresX = width/2;
     halfresY = height/2;

     // loadAnnotations(argv[2], locations);

     ofstream
          ofs(argv[2]);
     if (!ofs)
          errx(1,"Cannot open file %s", argv[4]);
     
     int key = 0;
     cout << "use ENTER to finish and save results" << endl ;
     while ((char)key != 'q' && index < imgs.size()) {
          key = cvWaitKey(0);
          switch ((char)key) {
          case 10:
               for (unsigned i=0; i!=locations.size(); ++i)
               {
					cout << locations[i].x << ","<< locations[i].y << endl;
                    ofs << locations[i].x << ","<< locations[i].y ;
                    ofs << endl;
               }
               locations.clear();
               cvReleaseImage(&img);

               index++;
               if (index==imgs.size())
                    goto stop;
               
               img = loadImage(imgs[index][0].c_str());
               if (!img)
                    errx(1,"Can't open %s", imgs[index][0].c_str());
               showScaledImg("image", img);
               
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
          case 'S':             // Arrow ->
               if (locations.size() == 0)
                    selected = 0;
               else 
                    selected = (selected+1)%locations.size();
               refresh();
               break;
          case 'Q':             // Arrow ->
               if (locations.size() == 0)
                    selected = 0;
               else  {
                    selected--;
                    if (selected>locations.size())
                         selected = locations.size()-1;
               }
               refresh();
               break;
          default:
               cout << "c = " << key << " char='" << (char)key << "'"<< endl;
          }
     }
     
stop:
     cvReleaseImage(&img);
     return 0;
}
