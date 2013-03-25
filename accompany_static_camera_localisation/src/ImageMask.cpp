
#include <ImageMask.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace std;

ImageMask::ImageMask()
{
  end();
  cout<<"construct: "<<*this<<endl;
}

void ImageMask::add(CvPoint p)
{
  currentRegion().push_back(p);
}

void ImageMask::end()
{
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
  CvScalar color=cvScalar(0,255,0);
  for (unsigned i=0;i<regions.size();i++)
  {
    CvPoint *points=&(regions[i][0]);
    int size=regions[i].size();
    bool closed=(i<regions.size()-1);
    cvPolyLine(img,&points,&size,1,closed,color);
  }
  vector<CvPoint>& cr=currentRegion();
  if (cr.size()>0)
    cvCircle(img,cr[cr.size()-1],10,color);
}

vector<CvPoint>& ImageMask::currentRegion()
{
  return regions[regions.size()-1];
}

std::ostream &operator<<(std::ostream &out,const CvPoint& point)
{
  out<<point.x<<","<<point.y;
  return out;
}

std::ostream &operator<<(std::ostream &out,const vector<CvPoint>& region)
{
  for (unsigned j=0;j<region.size();j++)
  {
    out<<region[j]<<" ";
  }
  return out;
}

/**
 * Allows ImageMask to be io streamed
 */
std::ostream &operator<<(std::ostream &out,const ImageMask& imageMask)
{
  out<<"ImageMask("<<imageMask.regions.size()<<"):"<<endl;
  for (unsigned i=0;i<imageMask.regions.size();i++)
    out<<i<<": "<<imageMask.regions[i]<<endl;
  return out;
}
