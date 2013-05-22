#ifndef AppearanceExtractor_INCLUDED
#define AppearanceExtractor_INCLUDED

#include <CamCalib.hh>
#include <Helpers.hh>
#include <Histogram.h>

#include <vector>
#include <vnl/vnl_vector.h>
#include <opencv/cv.h>

// Histogram template data
#define HIST_TYPE_DATA   unsigned char
#define HIST_TYPE_WEIGHT double
#define HIST_DIM    3
#define HIST_BINS   4
#define HIST_MIN    0
#define HIST_MAX  255
#define HISTOGRAM HistogramInt<HIST_TYPE_DATA,HIST_TYPE_WEIGHT,HIST_BINS,HIST_DIM,HIST_MIN,HIST_MAX>

/**
 * Deal with occulsions by foreground, allow detections that are closer to the camera to claim the pixels
 */
class PixelsClaimed
{
 public:
  
  PixelsClaimed(IplImage *image);
  unsigned char& operator[](int i);
  
 private:
  std::vector<unsigned char> pixels;

  void clear();
};

/**
 * Extract the appearance of each detection using last images of each cameras weighted by the background probablity
 */
class AppearanceExtractor
{
 public:

  vector<HISTOGRAM > computeAppearances(int c,
                                       const std::vector<CamCalib>& cam,
                                       const std::vector<unsigned>& existing,
                                       const vector<WorldPoint>& scanLocations,
                                       const std::vector<std::vector<std::vector<scanline_t> > > masks,
                                       std::vector<IplImage *> images,
                                       const std::vector<vnl_vector<FLOAT> >& bgProb);
  
  vector<HISTOGRAM > computeAppearances(const std::vector<CamCalib>& cam,
                                        const std::vector<unsigned>& existing,
                                        const vector<WorldPoint>& scanLocations,
                                        const std::vector<std::vector<std::vector<scanline_t> > > masks,
                                        std::vector<IplImage *> images,
                                        const std::vector<vnl_vector<FLOAT> >& bgProb);

 private:

  std::vector<int> orderDetections(const CamCalib& cam,
                                   const std::vector<unsigned>& existing,
                                   const vector<WorldPoint>& scanLocations);

  double squareDistance(const CamCalib& cam,
                        const WorldPoint& point);

};

#endif
