
#include <ImageMask.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace std;

ImageMask::ImageMask()
{
  regions.push_back(vector<CvPoint>());
}

void ImageMask::add(CvPoint p)
{
  currentRegion().push_back(p);
}

void ImageMask::end()
{
  if (currentRegion().size()>0) // can not end empty region
    regions.push_back(vector<CvPoint>());
}

void ImageMask::undo()
{
  if (currentRegion().size()>0)
    currentRegion().resize(currentRegion().size()-1); // remove last point of region
  else
  {
    if (regions.size()>1)
      regions.resize(regions.size()-1); // remove last region
  }
}

void ImageMask::plot(IplImage *img)
{
  CvScalar color[2];
  color[0]=cvScalar(0,255,0);
  color[1]=cvScalar(255,0,0);
  unsigned i;
  for (i=0;i<regions.size();i++)
  {
    CvPoint *points=&(regions[i][0]);
    int size=regions[i].size();
    bool closed=(i<regions.size()-1);
    cvPolyLine(img,&points,&size,1,closed,color[i%2],2);
  }
  vector<CvPoint>& cr=currentRegion();
  if (cr.size()>0)
    cvCircle(img,cr[cr.size()-1],10,color[(i+1)%2],2);
}

vector<CvPoint>& ImageMask::currentRegion()
{
  return regions[regions.size()-1];
}

/**
 * Allows ImageMask to be io streamed out
 */
/*
std::ostream &operator<<(std::ostream &out,const CvPoint& point) // not used this because of name clash in annotate_pos.cpp
{
  out<<point.x<<","<<point.y;
  return out;
}
*/
std::ostream& operator<<(std::ostream &out,const vector<CvPoint>& region)
{
  for (unsigned j=0;j<region.size();j++)
  {
    out<<region[j].x<<","<<region[j].y<<" ";
  }
  return out;
}

std::ostream& operator<<(std::ostream &out,const ImageMask& imageMask)
{
  out<<"ImageMask("<<imageMask.regions.size()<<"):"<<endl;
  for (unsigned i=0;i<imageMask.regions.size();i++)
    out<<"  "<<imageMask.regions[i].size()<<" "<<imageMask.regions[i]<<endl;
  return out;
}

/**
 * Allows ImageMask to be io streamed in
 */
std::istream& operator>>(std::istream &in,CvPoint& point)
{
  char comma;
  in>>point.x>>comma>>point.y;
  return in;
}

std::istream& operator>>(std::istream &in,vector<CvPoint>& region)
{
  int size;
  in>>size;
  for (int i=0;i<size;i++)
  {
    CvPoint point;
    in>>point;
    region.push_back(point);
  }

  return in;
}

std::istream& operator>>(std::istream &in,ImageMask& imageMask)
{
  if (imageMask.currentRegion().size()==0)
    imageMask.regions.resize(imageMask.regions.size()-1);
  string block;
  getline(in,block); // ignore first block

  while (true) // read all regions
  {
    getline(in,block);
    stringstream ss;
    ss<<block;
    vector<CvPoint> region;
    ss>>region;
    if (in.fail())
      break;
    else
      imageMask.regions.push_back(region);
  }

  return in;
}
