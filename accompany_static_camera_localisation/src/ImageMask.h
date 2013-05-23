#ifndef ImageMask_INCLUDED
#define ImageMask_INCLUDED

#include <opencv/cv.h>
#include <vector>

class ImageMask
{
 public:
  
  ImageMask();

  void add(CvPoint p);
  void end();
  void undo();

  void plot(IplImage *img);

  friend std::ostream &operator<<(std::ostream &out,const ImageMask& imageMask);
  friend std::istream &operator>>(std::istream &in,ImageMask& imageMask);

 private:
  
  std::vector< std::vector<CvPoint> > regions;

  std::vector<CvPoint>& currentRegion();

};

#endif
