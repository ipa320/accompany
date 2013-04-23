#ifndef CAM_CALIB_HH
#define CAM_CALIB_HH

#include <cameraModel.h>
#include <data/XmlPackable.hh>
#include <math.h>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "defines.hh"
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

#include <ImageMask.h>
#include <Hull.h>

// struct CvPoint;

extern double minX, maxX, minY, maxY;

class CamCalib: public XmlPackable
{
  public:
    Hu::CameraModel model;

    string name;
    string occlusionBGMaskFile;
    ImageMask occlusionBGMask;
    FLOAT stdev1, pixelVar1, pv2_1, lpv2_1;FLOAT stdev2, pixelVar2, pv2_2,
        lpv2_2;FLOAT stdev3, pixelVar3, pv2_3, lpv2_3;FLOAT scale;

    CamCalib();

    void updateCache()
    {
      pixelVar1 = stdev1 * stdev1;
      pv2_1 = 2 * pixelVar1;
      lpv2_1 = .5 * log(M_PI * pv2_1);
      pixelVar2 = stdev2 * stdev2;
      pv2_2 = 2 * pixelVar2;
      lpv2_2 = .5 * log(M_PI * pv2_2);
      pixelVar3 = stdev3 * stdev3;
      pv2_3 = 2 * pixelVar3;
      lpv2_3 = .5 * log(M_PI * pv2_3);
    }

    void xmlPack(XmlFile &f) const;
    void xmlUnpack(XmlFile &f);

    WorldPoint getGroundPos(const CvPoint &pt) const;
    CvPoint project(const WorldPoint &pt) const;
    void genTemplate(const WorldPoint &pt, std::vector<CvPoint> &points) const;
    void genFleuretTemplate(const WorldPoint &pt,
        std::vector<CvPoint> &points) const;

    // void img2vec(const IplImage *img, vnl_vector<FLOAT> &v);
    // IplImage *vec2img(const vnl_vector<FLOAT> &v) const;
    void computeBGProb(const vnl_vector<FLOAT> &img,
        const vnl_vector<FLOAT> &bg, vnl_vector<FLOAT> &sumPixelProb,
        FLOAT &total);
    void computeBGProbDiff(const vnl_vector<FLOAT> &img,
        const vnl_vector<FLOAT> &bg, vnl_vector<FLOAT> &sumPixelProb,
        FLOAT &total);
    void computeDiff(const vnl_vector<FLOAT> &img, const vnl_vector<FLOAT> &bg,
        vnl_vector<FLOAT> &diff);

};

extern std::vector<CamCalib> cam;
extern double personHeight, wg, wm, wt;

void plotTemplate(IplImage *img, const std::vector<CvPoint> &points,
    const CvScalar &colour, unsigned lw = 1);
void loadCalibrations(const char *filename);

void genScanLocations(const std::vector<WorldPoint> &prior, double scanRes,
    std::vector<WorldPoint> &sl);
void gridMatrix(const vnl_vector<FLOAT> &probs, vnl_matrix<FLOAT> &grid);
unsigned shiftedGridElt(unsigned elt, int shiftRow, int shiftCol);

// draw hull
void plotHull(IplImage *img, const vector<WorldPoint>& priorHull, unsigned idx, CvScalar color);
void plotHull(IplImage *img, const vector<WorldPoint>& priorHull, unsigned idx, CvScalar color, const WorldPoint &pt);

#endif  // CAM_CALIB_HH
