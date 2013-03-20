#ifndef AppearanceExtractor_INCLUDED
#define AppearanceExtractor_INCLUDED

#include <CamCalib.hh>
#include <Helpers.hh>

#include <vector>
#include <vnl/vnl_vector.h>
#include <opencv/cv.h>

/**
 * Deal with occulsions by foreground, allow detections that are closer to the camera to claim the pixels
 */
class PixelsClaimed
{
 public:
  
  void clear(IplImage *image);
  unsigned char& operator[](int i);
  
 private:
  std::vector<unsigned char> pixels;
  void clear();
};

/**
 * Extract the appearance of each detection using images from all cameras and weighted by the background probablity
 */
class AppearanceExtractor
{
 public:

  void computeAppearance(int c,
                         const std::vector<CamCalib>& cam,
                         const std::vector<unsigned>& existing,
                         const vector<WorldPoint>& scanLocations,
                         const std::vector<std::vector<std::vector<scanline_t> > > masks,
                         std::vector<IplImage *> images,
                         const std::vector<vnl_vector<FLOAT> >& bgProb);

  void computeAppearances(const std::vector<CamCalib>& cam,
                          const std::vector<unsigned>& existing,
                          const vector<WorldPoint>& scanLocations,
                          const std::vector<std::vector<std::vector<scanline_t> > > masks,
                          std::vector<IplImage *> images,
                          const std::vector<vnl_vector<FLOAT> >& bgProb);

 private:

  PixelsClaimed pixelsClaimed;

  // return indices that order 'existing' on distance to the camera
  std::vector<int> orderDetections(const CamCalib& cam,
                                   const std::vector<unsigned>& existing,
                                   const vector<WorldPoint>& scanLocations);

  // compute square distance of camera position to world point
  double squareDistance(const CamCalib& cam,
                        const WorldPoint& point);

};

#endif
