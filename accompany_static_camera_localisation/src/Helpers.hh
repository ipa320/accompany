#ifndef HELPERS_HH
#define HELPERS_HH

#include <vector>
#include <string>
#include <set>
#include <iostream>
#include <opencv/cv.h>
#include "defines.hh"

template<class T> class vnl_vector;

#ifndef HELPERS_CC
extern unsigned w, w2;
extern unsigned width, height, halfresX, halfresY, camPosX, camPosY;
extern int depth, channels;
extern unsigned scanres;
extern float camHeight, persHeight;
extern FLOAT stdev, pixelVar, pv2, lpv2;
extern FLOAT stdev1, pixelVar1, pv2_1, lpv2_1;
extern FLOAT stdev2, pixelVar2, pv2_2, lpv2_2;
extern FLOAT stdev3, pixelVar3, pv2_3, lpv2_3;
#endif // HELPERS_CC

void connectedComponents(std::vector<int> &v);
void connectedComponentsFH(std::vector<int> &v);
void convexHull(const std::vector<CvPoint> &pt, std::vector<CvPoint> &hull);
void plotHull(IplImage *img, const std::vector<CvPoint> &tpl);
void genTemplate(const CvPoint &pt, float persHeight, float camHeight,
    std::vector<CvPoint> &points);
void plotTemplate(IplImage *img, const CvPoint &pt, float persHeight,
    float camHeight, const CvScalar &colour = CV_RGB(255,255,255));
void genTemplate2(const CvPoint &pt, float persHeight, float camHeight,
    std::vector<CvPoint> &points);
void plotTemplate2(IplImage *img, const CvPoint &pt, float persHeight,
    float camHeight, const CvScalar &colour = CV_RGB(255,255,255));
CvPoint project(float x, float y, float z, float camHeight);

void getMask(const CvPoint &pos, float persHeight, float camHeight,
    std::vector<int> &mask);
void getMask(const CvPoint &pos, float persHeight, float camHeight,
    std::set<unsigned> &mask);

struct scanline_t
{
    unsigned line, start, end;
};
std::ostream &operator<<(std::ostream &os, const scanline_t &l);
void getMask(const CvPoint &pos, float persHeight, float camHeight,
    std::vector<scanline_t> &mask);
void getMask(const std::vector<CvPoint> &tpl, std::vector<scanline_t> &mask);
void mergeMasks(std::vector<scanline_t> &v1, std::vector<scanline_t> &v2);
void mergeMasksRest(std::vector<scanline_t> &v1, std::vector<scanline_t> &v2);
bool masksOverlap(const std::vector<scanline_t> &m1,
    const std::vector<scanline_t> &m2);

CvPoint toCam(CvPoint p);
CvPoint toImg(CvPoint p);
void listImages(const char *list, std::vector<std::string> &imgs);
void listImages(const char *list, std::vector<std::vector<std::string> > &imgs);
void listImages(const char *list, std::vector<std::string> &imgs,
    std::vector<double> &tstamps);
void applyMask(IplImage *img, const std::vector<int> &mask);
float loadPriorHull(const char *file, std::vector<CvPoint> &polygon);
float loadPrior(const char *file, std::vector<scanline_t> &prior);
void loadPrior(const char *file, vnl_vector<FLOAT> &prior);
void plotScanLines(IplImage *img, const std::vector<scanline_t> &mask,
    const CvScalar &colour, double alpha);

void saveCalibration(const char *fileName);
void loadCalibration(const char *fileName);

void medianFilter(vnl_vector<FLOAT> &v, unsigned w);
IplImage *loadImage(const char *filename);

#endif  // HELPERS_HH
