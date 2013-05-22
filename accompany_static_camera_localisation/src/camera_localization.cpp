
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <stdio.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include "fstream"

#include <boost/thread/xtime.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#if BOOST_VERSION < 103500
#include <boost/thread/detail/lock.hpp>
#endif
#include <boost/thread.hpp>  
#include <boost/date_time.hpp> 

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>

#include <accompany_uva_utils/uva_utils.h>
#include <accompany_uva_msg/HumanDetections.h>
#include <accompany_uva_msg/MsgToMarkerArray.h>

#include "cmn/FastMath.hh"
#include "cmn/random.hh"
#include "tools/utils.hh"
#include "tools/string.hh"
#include "Helpers.hh"
#include "Background.hh"
#include <data/XmlFile.hh>
#include "ImgProducer.hh"
#include "CamCalib.hh"

#include "AppearanceExtractor.h"

namespace po = boost::program_options;
using namespace std;
using namespace cv;

unsigned MIN_TRACK_LEN = 20;
unsigned MAX_TRACK_JMP = 1000;
unsigned MAX_TRACK_AGE = 8;

#define USE_DYNAMIC_BACKGROUND 1 // switch between static (PCA) and dynamic (gaussian mixture) background model

unsigned int CAM_NUM = 2;

// ---- dynamic background model ---- start
#include <GaussianMixture.h>
#include <Gaussian.h>
#include <GaussianSpherical.h>
#include <GaussianSphericalCone.h>
using namespace GausMix;
#define DYNBG_TYPE float
#define DYNBG_DIM 3
#define DYNBG_GAUS GaussianSphericalCone<DYNBG_TYPE,DYNBG_DIM>
#define DYNBG_MAXGAUS 3
#define INIT_FIRST_FRAMES 1 // use first X frames to initializa the background model
vector<GaussianMixture<DYNBG_GAUS,DYNBG_TYPE,DYNBG_MAXGAUS> *> gaussianMixtures(CAM_NUM);
DYNBG_TYPE decay=1/800.0f;
DYNBG_TYPE initVar=5;
DYNBG_TYPE minWeight=.2;
DYNBG_TYPE squareMahanobisMatch=10;
DYNBG_TYPE weightReduction=0.008;
const char* dynBGProb = "Background Probability";

// vizualize
//vnl_vector<FLOAT> bgProb;
FLOAT bgProbMin,bgProbMax;
// ---- dynamic background model ---- end

extern unsigned w2;
vector<Background> bgModel(0, 0);
vector<FLOAT> logNumPrior;
vnl_vector<FLOAT> logLocPrior;
vector<WorldPoint> priorHull;
vector<vnl_vector<FLOAT> > logSumPixelFGProb;
vector<vector<vector<scanline_t> > > masks; // camera, id, lines
vector<WorldPoint> scanLocations;
MsgToMarkerArray msgToMarkerArray;

bool running=true;
geometry_msgs::TransformStamped frame;
tf::TransformBroadcaster *transformBroadcasterPtr;
ros::Publisher humanDetectionsPub,
                    humanLocationsParticlesPub,
                    markerArrayPub;

unsigned int NUM_PERSONS;
bool HAS_INIT = false;
vector<IplImage*> cvt_vec(CAM_NUM);
vector<vnl_vector<FLOAT> > img_vec(CAM_NUM), bg_vec(CAM_NUM);
vector<IplImage *> src_vec(CAM_NUM);
vector<vnl_vector<FLOAT> > bgProb(CAM_NUM);
vector<vnl_vector<FLOAT> > sumPixel(CAM_NUM);
vector<FLOAT> sum_g(CAM_NUM);

unsigned nrThreads;

// particle options
bool particles = false;
int nrParticles = 10;
int max = 100;
int frame_cnt = 0;
int cum_cnt = 0;
int direction = 1;

ImgProducer *producer;

int waitTime = 30;
int visualize;

string save_all;
char image_name[200];
char data_file[200];
Mat frame_id, cum_id, human_locations, human_templates; // buffer to store detection results
FileStorage fs;

// publish debug frames
image_transport::Publisher *backgroundsPub;
image_transport::Publisher *detectionsPub;

AppearanceExtractor appearanceExtractor;

// counts the frames of each camera
class FrameCounter
{
private:
  vector<int> counters;
  int currentCount;
public:
  FrameCounter(int camNum)
  {
    currentCount=0;
    counters.resize(camNum);
  }

  // returns the smallest frame count when all cameras have provided a new frame since the last successful call
  // otherwise return -1;
  int count(int c)
  {
    if (counters[c]<=currentCount)
      counters[c]=currentCount+1;
    int upCount=currentCount+1; // try to add 1
    for (vector<int>::iterator it=counters.begin();it!=counters.end();it++)
    {
      if (*it<upCount) // failed
      {
        upCount=-1;
        break;
      }
    }
    if (upCount>0) // success
      currentCount=upCount;
    return upCount;
  }

};
FrameCounter *frameCounter;

void buildMasks()
{
  masks = vector<vector<vector<scanline_t> > >(cam.size(),
      vector<vector<scanline_t> >(scanLocations.size()));

  for (unsigned n = 0; n != cam.size(); ++n)
    for (unsigned i = 0; i != scanLocations.size(); ++i)
    {
      vector<CvPoint> tpl;
      cam[n].genTemplate(scanLocations[i], tpl);
      getMask(tpl, masks[n][i]);
    }
}

FLOAT logMaskProb(const vnl_vector<FLOAT> &logSumPixelFGProb,
    const vnl_vector<FLOAT> &logSumPixelBGProb, FLOAT bgSum,
    const vector<scanline_t> &mask)
{
  // cerr << "logSumPixelFGProb.size()=" << logSumPixelFGProb.size() << ", logSumPixelBGProb.size()="
  //      << logSumPixelBGProb.size() << ", mask.size()=" << mask.size() << endl;
  // cerr << "logSumPixelFGProb.size()=" << logSumPixelFGProb.size() << endl;
  FLOAT res = bgSum;
  for (vector<scanline_t>::const_iterator i = mask.begin(); i != mask.end();++i)
  {
    unsigned offset = i->line * (width + 1) + 1; // if start=0, offset-1 == 0
    //cout << "offset=" << offset << ", start=" << i->start << ", end=" << i->end << ", size=" << logSumPixelBGProb.size() << endl;
    
    res -= (logSumPixelBGProb(offset + i->end)
            - logSumPixelBGProb(offset + i->start - 1));
    // res += (logSumPixelFGProb(offset + i->end) - logSumPixelFGProb(offset + i->start - 1))
    //      - (logSumPixelBGProb(offset + i->end) - logSumPixelBGProb(offset + i->start - 1));
  }
  return res;
}

FLOAT logMaskProbDiff(const vnl_vector<FLOAT> &logSumPixelFGProb,
    const vnl_vector<FLOAT> &logSumPixelBGProb, const vector<scanline_t> &mask)
{
  FLOAT res = 0;
  for (vector<scanline_t>::const_iterator i = mask.begin(); i != mask.end();++i)
  {
    unsigned offset = i->line * (width + 1) + 1; // if start=0, offset-1 == 0
    res -= (logSumPixelBGProb(offset + i->end)
        - logSumPixelBGProb(offset + i->start - 1));
  }

  return res;
}

/**
 * \brief Do ground planes overlap?
 * \param existing
 * \param x
 * \param y
 * \return
 *
 * 2010/06/11: GWENN - First version
 *
 **/bool overlap(const vector<unsigned> &existing, unsigned pos)
{
  WorldPoint &p2 = scanLocations[pos];
  for (unsigned i = 0; i != existing.size(); ++i)
  {
    WorldPoint &pt = scanLocations[existing[i]];
    if (p2.x >= pt.x - w2 && p2.x <= pt.x + w2 && p2.y >= pt.y - w2
        && p2.y <= pt.y + w2)
      return true;
  }
  return false;
}

class ScanResResult
{
public:
  ScanResResult()
  {
    bestIdx = 0;
    bestLogProb = -INFINITY;
  }
  
  FLOAT bestLogProb;
  unsigned bestIdx;
};

class ScanRestHelperThreadSuper
{
public:
  ScanRestHelperThreadSuper(unsigned begin,
                            unsigned end,
                            vector<unsigned> &existing,
                            vector<vector<scanline_t> > &existingMask,
                            const vector<vnl_vector<FLOAT> > &logSumPixelFGProb,
                            const vector<vnl_vector<FLOAT> > &logSumPixelBGProb,
                            const vector<FLOAT> &lSum,
                            vector<FLOAT> &logPosProbCache,
                            FLOAT logNP,
                            vnl_vector<FLOAT> &lpp)
    : begin(begin),end(end),existing(existing),existingMask(existingMask),
      logSumPixelFGProb(logSumPixelFGProb),logSumPixelBGProb(logSumPixelBGProb),
      lSum(lSum),logPosProbCache(logPosProbCache),logNP(logNP),lpp(lpp)
  {
  }

protected:
  unsigned begin;
  unsigned end;
  vector<unsigned>& existing;
  vector<vector<scanline_t> >& existingMask;
  const vector<vnl_vector<FLOAT> >& logSumPixelFGProb;
  const vector<vnl_vector<FLOAT> >& logSumPixelBGProb;
  const vector<FLOAT>& lSum;
  vector<FLOAT>& logPosProbCache;
  FLOAT logNP;
  vnl_vector<FLOAT>& lpp;
};

class ScanRestHelper1Thread : public ScanRestHelperThreadSuper
{
public:
  ScanRestHelper1Thread(unsigned begin,
                        unsigned end,
                        vector<unsigned> &existing,
                        vector<vector<scanline_t> > &existingMask,
                        const vector<vnl_vector<FLOAT> > &logSumPixelFGProb,
                        const vector<vnl_vector<FLOAT> > &logSumPixelBGProb,
                        const vector<FLOAT> &lSum,
                        vector<FLOAT> &logPosProbCache,
                        FLOAT logNP,
                        vnl_vector<FLOAT> &lpp)
    : ScanRestHelperThreadSuper(begin,end,existing,existingMask,logSumPixelFGProb,logSumPixelBGProb,
                                lSum,logPosProbCache,logNP,lpp)
  {
  }

  ScanResResult operator()()
  {
    ScanResResult scanResResult;
    for (unsigned p = begin; p<end; ++p)
    {
      if (!overlap(existing, p) && logPosProbCache[p] > 0)
      {
        lpp(p) = logLocPrior(p) + logNP;
        for (unsigned c = 0; c != cam.size(); ++c)
        {
          vector<scanline_t> mask = existingMask[c];
          mergeMasks(mask, masks[c][p]);
          lpp(p) += logMaskProb(logSumPixelFGProb[c], logSumPixelBGProb[c],
                                   lSum[c], mask);
        }
        if (lpp(p) > scanResResult.bestLogProb)
        {
          scanResResult.bestLogProb = lpp(p);
          scanResResult.bestIdx = p;
        }
      }
    }
    return scanResResult;
  }

};

class ScanRestHelper2Thread : public ScanRestHelperThreadSuper
{
public:
  ScanRestHelper2Thread(unsigned begin,
                        unsigned end,
                        vector<unsigned> &existing,
                        vector<vector<scanline_t> > &existingMask,
                        const vector<vnl_vector<FLOAT> > &logSumPixelFGProb,
                        const vector<vnl_vector<FLOAT> > &logSumPixelBGProb,
                        const vector<FLOAT> &lSum,
                        vector<FLOAT> &logPosProbCache,
                        FLOAT logNP,
                        vnl_vector<FLOAT> &lpp)
    : ScanRestHelperThreadSuper(begin,end,existing,existingMask,logSumPixelFGProb,logSumPixelBGProb,
                                lSum,logPosProbCache,logNP,lpp)
  {
  }

  ScanResResult operator()()
  {
    ScanResResult scanResResult;
    for (unsigned p = begin; p<end; ++p)
    {
      logPosProbCache[p] = logLocPrior(p);
      for (unsigned c = 0; c != cam.size(); ++c)
      {
        logPosProbCache[p] += logMaskProb(logSumPixelFGProb[c],
                                          logSumPixelBGProb[c], lSum[c], masks[c][p]); // already includes lSum
      }
      lpp(p) = logPosProbCache[p] + logNP;
      logPosProbCache[p] -= logNumPrior[0];
      for (unsigned c = 0; c != cam.size(); ++c)
      {
        logPosProbCache[p] -= lSum[c]; // hack -> if pos, increases lik; if neg, doesn't
      }
      if (lpp(p) > scanResResult.bestLogProb)
      {
        scanResResult.bestLogProb = lpp(p);
        scanResResult.bestIdx = p;
      }
    }
    return scanResResult;
  }

};

/**
 * \brief Recursively scan the image for a number of people
 * \param existing How many people have been scanned for yet
 * \param logFGProb The array of foreground log-probabilities
 * \param logBGProb The array of background log-probabilities
 * \param logPosProb The array containing the
 * \param bestPoint
 * \param scanPos
 * \param lSum
 *
 * 2010/03/25: GWENN - First version
 *
 **/
void scanRest(vector<unsigned> &existing,
    vector<vector<scanline_t> > &existingMask,
    const vector<vnl_vector<FLOAT> > &logSumPixelFGProb,
    const vector<vnl_vector<FLOAT> > &logSumPixelBGProb,
    const vector<FLOAT> &logNumPrior,
    //              const vnl_vector<FLOAT> &logLocPrior, // remove?
    vector<vnl_vector<FLOAT> > &logPosProb, vector<FLOAT > &marginal,
    const vector<FLOAT> &lSum)
{
  unsigned bestIdx = 0;
  FLOAT bestLogProb = -INFINITY;//, marginalLogProb = -INFINITY; //not used?

  logPosProb.push_back(vnl_vector<FLOAT>(scanLocations.size(), -INFINITY));
  vnl_vector<FLOAT> &lpp = logPosProb.back();

  static vector<FLOAT> logPosProbCache(scanLocations.size(), -INFINITY);

  // cout << "marginal=[ ";
  // for (unsigned i=0; i!=marginal.size(); ++i)
  //      cout << marginal[i] << " ";
  // cout << "]" << endl;

  if (existing.size())
  {
    unsigned &hereIdx = existing.back();
    for (unsigned c = 0; c != cam.size(); ++c)
      mergeMasks(existingMask[c], masks[c][hereIdx]);

    FLOAT logNP = logNumPrior[existing.size() + 1];

    // threaded version
    boost::unique_future<ScanResResult> futures[nrThreads];
    for (unsigned i=0;i<nrThreads;i++)
    {
      unsigned begin=i*(scanLocations.size()/nrThreads);
      unsigned end=(i+1)*(scanLocations.size()/nrThreads);
      if (i==nrThreads-1)
        end=scanLocations.size();
      //helpers[i]=.
      ScanRestHelper1Thread helper(begin,end,existing,existingMask,logSumPixelFGProb,logSumPixelBGProb,lSum,
                                   logPosProbCache,logNP,lpp);

      boost::packaged_task<ScanResResult> pt(helper);
      futures[i]=pt.get_future();
      boost::thread thread(boost::move(pt)); // start thread
    }
    for (unsigned i=0;i<nrThreads;i++)
    {
      futures[i].wait(); // join thread
      ScanResResult scanResResult=futures[i].get();
      if (scanResResult.bestLogProb>bestLogProb)
      {
        bestLogProb=scanResResult.bestLogProb;
        bestIdx=scanResResult.bestIdx;
      }
    }

    /* // non-threaded version
    for (unsigned p = 0; p != scanLocations.size(); ++p)
    {
      if (!overlap(existing, p) && logPosProbCache[p] > 0)
      {
        // cout << "Considering position " << p << ": (" << scanLocations[p].x << "," << scanLocations[p].y << ")" <<endl;
        // if (logLocPrior(p) < -5000)  --> Not part of scanLocations anymore
        //      continue;
        lpp(p) = logLocPrior(p) + logNP;
        for (unsigned c = 0; c != cam.size(); ++c)
        {
          vector<scanline_t> mask = existingMask[c];
          mergeMasks(mask, masks[c][p]);
          lpp(p) += logMaskProb(logSumPixelFGProb[c], logSumPixelBGProb[c],
                                lSum[c], mask);
        }
        //marginalLogProb = log_sum_exp(marginalLogProb, lpp(p));
        if (lpp(p) > bestLogProb)
        {
          bestLogProb = lpp(p);
          bestIdx = p;
        }
      }
    }
    */
  }
  else
  {
    FLOAT logNP = logNumPrior[1];

    
    // threaded version
    boost::unique_future<ScanResResult> futures[nrThreads];
    for (unsigned i=0;i<nrThreads;i++)
    {
      unsigned begin=i*(scanLocations.size()/nrThreads);
      unsigned end=(i+1)*(scanLocations.size()/nrThreads);
      if (i==nrThreads-1)
        end=scanLocations.size();
      //helpers[i]=
      ScanRestHelper2Thread helper(begin,end,existing,existingMask,logSumPixelFGProb,logSumPixelBGProb,lSum,
                                   logPosProbCache,logNP,lpp);

      boost::packaged_task<ScanResResult> pt(helper);
      futures[i]=pt.get_future();
      boost::thread thread(boost::move(pt)); // start thread
    }
    for (unsigned i=0;i<nrThreads;i++)
    {
      futures[i].wait(); // join thread
      ScanResResult scanResResult=futures[i].get();
      cout<<"=== bestLogProb:"<<scanResResult.bestLogProb<<endl;
      if (scanResResult.bestLogProb>bestLogProb)
      {
        bestLogProb=scanResResult.bestLogProb;
        bestIdx=scanResResult.bestIdx;
      }
    }

    /* // non-threaded version
    for (unsigned p = 0; p != scanLocations.size(); ++p)
    {
      // if (logLocPrior(p) < -5000) --> Not part of scanLocations anymore
      //      continue;

      logPosProbCache[p] = logLocPrior(p);
      for (unsigned c = 0; c != cam.size(); ++c)
      {
        logPosProbCache[p] += logMaskProb(logSumPixelFGProb[c],
            logSumPixelBGProb[c], lSum[c], masks[c][p]); // already includes lSum
      }
      lpp(p) = logPosProbCache[p] + logNP;
      logPosProbCache[p] -= logNumPrior[0];
      for (unsigned c = 0; c != cam.size(); ++c)
      {
        logPosProbCache[p] -= lSum[c]; // hack -> if pos, increases lik; if neg, doesn't
      }
      //marginalLogProb = log_sum_exp(marginalLogProb, lpp(p));
      if (lpp(p) > bestLogProb)
      {
        bestLogProb = lpp(p);
        bestIdx = p;
      }
    }
    */
  }
  // cout << "bestIdx=" << bestIdx << ", bestLogProb=" << bestLogProb << endl;
  //      if (marginalLogProb > marginal.back()) {
  if (bestLogProb > marginal.back())
  {
    existing.push_back(bestIdx);
    // marginal.push_back(marginalLogProb);
    marginal.push_back(bestLogProb);

    if (existing.size() > NUM_PERSONS - 1 && NUM_PERSONS > 0) // TODO
      return;

    scanRest(existing, existingMask, logSumPixelFGProb, logSumPixelBGProb,
        logNumPrior, /*logLocPrior, */logPosProb, marginal, lSum);
  }
}

CvPoint cvPoint(unsigned pos)
{
  return cvPoint(pos % width, pos / width);
}

void plotHull(IplImage *img, vector<WorldPoint> &hull, unsigned c)
{
  hull.push_back(hull.front());
  for (unsigned i = 1; i < hull.size(); ++i)
    cvLine(img, cam[c].project(hull[i - 1]), cam[c].project(hull[i]),
        CV_RGB(255,0,0), 2);
}

void save_detection_results(vector<unsigned> existing)
{
  cum_cnt += existing.size();
  Mat increment_loc = Mat::zeros(existing.size(),1,CV_64FC2);
  Mat increment_tplt = Mat::zeros(existing.size(),24,CV_64FC2);

  for (unsigned i = 0; i != existing.size(); ++i)
  {
    WorldPoint wp = scanLocations[existing[i]];
    Point2d human_location_2D = cam[0].project(scanLocations[existing[i]]);
    increment_loc.at<Point2d>(i,0) =  human_location_2D;
//    increment_loc.at<double>(i,0) =  wp.x/1000;
//    increment_loc.at<double>(i,1) =  wp.y/1000;

    vector<CvPoint> tplt;
    cam[0].genTemplate(wp, tplt);

    vector<Point2d> reload_tplt;
    for (unsigned iter_tplt=0; iter_tplt != tplt.size(); iter_tplt++)
    {
      reload_tplt.push_back(tplt[iter_tplt]);
    }
//    cout << "---" << endl;

    Mat iP(reload_tplt);
    CV_Assert(iP.depth() == increment_tplt.depth());
    iP = iP.t();
    Mat temp=increment_tplt.row(i);
    iP.copyTo(temp);
    frame_id.push_back(frame_cnt);
  }

  if (cum_id.empty())
    cum_id.push_back(0);
  cum_id.push_back(cum_cnt);
  human_locations.push_back(increment_loc);
  human_templates.push_back(increment_tplt);

  fs.open(data_file, FileStorage::WRITE);
  fs << "frame_id" << frame_id;
  fs << "cum_id" << cum_id;
  fs << "human_locations" << human_locations;
  fs << "human_templates" << human_templates;
  fs.release();
}

void save_image_frames(IplImage* oriImage)
{
    sprintf(image_name,"%s/%04d.jpg",save_all.c_str(),frame_cnt);
    imwrite(image_name,cvarrToMat(oriImage));
}

void save_background_frames(IplImage* oriImage)
{
    sprintf(image_name,"%s/bg%04d.jpg",save_all.c_str(),frame_cnt);
    imwrite(image_name,cvarrToMat(oriImage));
}

accompany_uva_msg::HumanDetections findPerson(unsigned imgNum,
    vector<IplImage *> src, const vector<vnl_vector<FLOAT> > &imgVec,
    vector<vnl_vector<FLOAT> > &bgVec, const vector<FLOAT> logBGProb,
    const vector<vnl_vector<FLOAT> > &logSumPixelFGProb,
    const vector<vnl_vector<FLOAT> > &logSumPixelBGProb)
{
  // stic();

  vector<FLOAT> marginal;
  FLOAT lNone = logNumPrior[0];

  for (unsigned c = 0; c != cam.size(); ++c)
    lNone += logBGProb[c];
  FLOAT lSum = -INFINITY;

  vector<unsigned> existing;
  vector<vnl_vector<FLOAT> > logPosProb;
  vector<vector<scanline_t> > mask(cam.size());

  marginal.push_back(lNone);
  logPosProb.push_back(vnl_vector<FLOAT>());
  // cout << "Marginal[0]=" << marginal.back() << endl;
  scanRest(existing, mask, logSumPixelFGProb, logSumPixelBGProb, logNumPrior,
  /*logLocPrior, */logPosProb, marginal, logBGProb);

  // compute histograms
  vector<HISTOGRAM > appearances=appearanceExtractor.computeAppearances(cam,
                                                                        existing,
                                                                        scanLocations,
                                                                        masks,
                                                                        src,
                                                                        bgProb);
  for (unsigned i=0;i<appearances.size();i++)
  {
    cout<<"appearance "<<i<<" "<<appearances[i]<<endl;
  }
  // report detections
  accompany_uva_msg::HumanDetections humanDetections;
  geometry_msgs::PointStamped v;
  std_msgs::Header header;
  header.stamp=ros::Time::now(); 
  header.frame_id=frame.child_frame_id;
  v.header=header;// set current time and frame name to the point

  cout<<"locations found are:"<<endl;
  for (unsigned i = 0; i != existing.size(); ++i)
  {
    // add human detection
    accompany_uva_msg::HumanDetection humanDetection;
    WorldPoint wp = scanLocations[existing[i]];
    cout<<" "<<scanLocations[existing[i]];
    v.point.x = wp.x/1000; // millimeters to meters
    v.point.y = wp.y/1000;
    v.point.z = 0;
    humanDetection.location=v;
    humanDetection.appearance.sumTemplatePixelSize=appearances[i].getAddCount();
    humanDetection.appearance.sumPixelWeights=appearances[i].getWeightCount();
    humanDetection.appearance.histogram=appearances[i].normalize();
    humanDetections.detections.push_back(humanDetection);
  }
  cout<<endl<<"==========="<<endl;

  for (unsigned i = 0; i != marginal.size(); ++i)
    lSum = log_sum_exp(lSum, marginal[i]);
  FLOAT mlprob = -INFINITY;
  for (unsigned i = 0; i != marginal.size(); ++i)
  {
    // cout << "p(n=" << i << ") = " << exp(marginal[i]-lSum)
    //      << " (log = " << marginal[i] << ")"
    //      << endl;
    if (marginal[i] > mlprob)
      mlprob = marginal[i];
  }

  // SAVE Locations and template points
  if (!save_all.empty()) // save all images
    save_detection_results(existing);

  static int number = 0;
  number++;

  if (visualize)
  {
    /* Visualize tracks */
    for (unsigned c = 0; c != cam.size(); ++c)
    {
      plotHull(src[c], priorHull, c);
      for (unsigned i = 0; i != existing.size(); ++i)
      {
        vector<CvPoint> tplt;
        cam[c].genTemplate(scanLocations[existing[i]], tplt);
        //plotTemplate(cvt, tplt, CV_RGB(255,255,255));
        plotTemplate(src[c], tplt, CV_RGB(0,255,255));
        cvCircle(src[c], cam[c].project(scanLocations[existing[i]]), 1,
                 CV_RGB(255,255,0), 2);
      }

      // convert to Ros image and publish
      cv_bridge::CvImage detectImgRos;
      detectImgRos.encoding = "bgr8";
      detectImgRos.image = src[c];
      sensor_msgs::ImagePtr imagePtr=detectImgRos.toImageMsg();
      detectionsPub[c].publish(imagePtr);
    }
  }
  /* End of Visualization */

  return humanDetections;
}

void initStaticProbs()
{
  unsigned maxN = 50;
  logNumPrior = vector<FLOAT>(maxN, -log((FLOAT) maxN));

  logNumPrior.push_back(-INFINITY); // p(#=LAST)

  unsigned wp1 = width + 1;
  logSumPixelFGProb.resize(cam.size());
  logSumPixelFGProb[0] = vnl_vector<FLOAT>((wp1) * height);
  FLOAT U = -3.0 * log(256.0);
  for (unsigned i = 0; i != logSumPixelFGProb[0].size(); ++i)
  {
    //      if (i%(width+1) == 0)
    //           logSumPixelFGProb(i) = 0;
    //      else
    //           logSumPixelFGProb(i) = logSumPixelFGProb(i-1) + U;
    logSumPixelFGProb[0](i) = (i % (wp1)) * U;
  }
  for (unsigned c = 1; c != cam.size(); ++c)
    logSumPixelFGProb[c] = logSumPixelFGProb[0];
}

class DynBackGroundThread
{
public:
  DynBackGroundThread(unsigned begin,unsigned end,
                      IplImage *smooth,
                      GaussianMixture<DYNBG_GAUS,DYNBG_TYPE,DYNBG_MAXGAUS> *gaussianMixtures,
                      vnl_vector<FLOAT> &bgProbRef)
  : bgProb(bgProbRef)
  {
    this->begin=begin;
    this->end=end;
    this->smooth=smooth;
    this->gaussianMixtures=gaussianMixtures;
  }

  void operator()()
  {
    int updateGaussianID;
    DYNBG_TYPE data[DYNBG_DIM],squareDist[DYNBG_DIM];
    int channelInd=begin*3;
    for (unsigned int i=begin;i<end;i++)
    {
      data[0]=(unsigned char)(smooth->imageData[channelInd+0]);
      data[1]=(unsigned char)(smooth->imageData[channelInd+1]);
      data[2]=(unsigned char)(smooth->imageData[channelInd+2]);
      DYNBG_TYPE logProbBG=gaussianMixtures[i].logProbability(data,squareDist,minWeight,squareMahanobisMatch,updateGaussianID);
      gaussianMixtures[i].update(data,initVar,decay,weightReduction,updateGaussianID);
      bgProb(i)=logProbBG;
      if (logProbBG<bgProbMin) bgProbMin=logProbBG;
      if (logProbBG>bgProbMax) bgProbMax=logProbBG;
      channelInd+=3; // next pixel
    }
  }

private:
  unsigned begin,end;
  IplImage *smooth;
  GaussianMixture<DYNBG_GAUS,DYNBG_TYPE,DYNBG_MAXGAUS> *gaussianMixtures;
  vnl_vector<FLOAT> &bgProb;
};

void getDynamicBackgroundLogProb(IplImage *smooth,
                                 GaussianMixture<DYNBG_GAUS,DYNBG_TYPE,DYNBG_MAXGAUS> *gaussianMixtures,
                                 vnl_vector<FLOAT> &bgProb)
{
  unsigned int size=smooth->width*smooth->height;
  if (bgProb.size()!=size)
    bgProb.set_size(size);
  bgProbMin=std::numeric_limits<float>::max();
  bgProbMax=-std::numeric_limits<float>::max();

  // threaded version
  boost::thread threads[nrThreads];
  for (unsigned i=0;i<nrThreads;i++)
  {
    unsigned begin=i*(size/nrThreads);
    unsigned end=(i+1)*(size/nrThreads);
    if (i==nrThreads-1) // is last
      end=size;
    threads[i]=boost::thread(DynBackGroundThread(begin,end,
                                                 smooth,gaussianMixtures,bgProb));
  }
  for (unsigned i=0;i<nrThreads;i++)
    threads[i].join();

  /* // non-threaded version
  int updateGaussianID;
  DYNBG_TYPE data[DYNBG_DIM],squareDist[DYNBG_DIM];
  int channelInd=0;
  for (unsigned int i=0;i<size;i++)
  {
    data[0]=(unsigned char)(smooth->imageData[channelInd+0]);
    data[1]=(unsigned char)(smooth->imageData[channelInd+1]);
    data[2]=(unsigned char)(smooth->imageData[channelInd+2]);
    DYNBG_TYPE logProbBG=gaussianMixtures[i].logProbability(data,squareDist,minWeight,squareMahanobisMatch,updateGaussianID);
    gaussianMixtures[i].update(data,initVar,decay,weightReduction,updateGaussianID);
    bgProb(i)=logProbBG;
    if (logProbBG<bgProbMin) bgProbMin=logProbBG;
    if (logProbBG>bgProbMax) bgProbMax=logProbBG;
    channelInd+=3; // next pixel
  }
  */
}

void getDynamicBackgroundSumLogProb(IplImage *smooth,
                                    const vnl_vector<FLOAT> &bgProb,
                                    vnl_vector<FLOAT> &sumPix,
                                    FLOAT &sum)
{
  unsigned int widthExtra=smooth->width+1; // each line needs an extra leading zero
  unsigned int imgSizeExtra=widthExtra*smooth->height;

  if (sumPix.size()!=imgSizeExtra)
    sumPix.set_size(imgSizeExtra);

  int pixelInd=0,channelInd=0;
  sum=0;
  for (unsigned int i=0;i<imgSizeExtra;i++)
  {
    if (i%widthExtra==0) // add leading zero
      sumPix(i)=0;
    else
    {
      sumPix(i)=sumPix(i-1)+bgProb(pixelInd)+3.0*log(256); // - (-log ...) , something to do with foreground probablity
      sum+=bgProb(pixelInd);
      pixelInd+=1;channelInd+=3; // next pixel
    }
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
  ROS_INFO_STREAM("imageCallback frame:"<<image->header.frame_id);
  unsigned c=0;

  for (unsigned int i=0;i<cam.size();i++)
    if (image->header.frame_id.compare(cam[i].name)==0)
      c=i;

  ROS_INFO_STREAM("camera:"<<c);

  try
  {
    frame_cnt ++ ;
    // load the first image to get image size
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(image,"bgr8");
    IplImage helpConvert=(IplImage)(cv_ptr->image);
    IplImage *oriImage=&helpConvert;

    if (!save_all.empty())
      save_image_frames(oriImage);

    if (src_vec[c]!=NULL)
      cvReleaseImage(&src_vec[c]);
    src_vec[c] = cvCloneImage(oriImage);

    if (!HAS_INIT)
    {
      width = src_vec[c]->width;
      height = src_vec[c]->height;
      depth = src_vec[c]->depth;
      channels = src_vec[c]->nChannels;
      halfresX = width / 2;
      halfresY = height / 2;

      initStaticProbs();
      buildMasks();

      for (unsigned cc=0; cc!=cam.size(); cc++)
      {
        if (!src_vec[c])
        {
          ROS_ERROR("empty image frame");
        }
        cvt_vec[cc] = cvCreateImage(cvGetSize(src_vec[c]), IPL_DEPTH_8U, 3);
        gaussianMixtures[cc]=new GaussianMixture<DYNBG_GAUS,DYNBG_TYPE,DYNBG_MAXGAUS>[width*height];
      }
      HAS_INIT = true;
    }


#if USE_DYNAMIC_BACKGROUND
    IplImage *smooth=cvCloneImage(oriImage);
    cvSmooth(smooth, smooth, CV_GAUSSIAN, 7, 7, 0, 0); // smooth to improve background estimation
    getDynamicBackgroundLogProb(smooth,gaussianMixtures[c],bgProb[c]); // compute bgProb
    getDynamicBackgroundSumLogProb(smooth,bgProb[c],sumPixel[c],sum_g[c]); // compute the sums

    IplImage *bgProbImg;
    if (visualize)
    {
      bgProbImg=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
      int size=width*height;
      bgProbMin=-1000;// improves visualisation
      for (int i=0;i<size;i++)
      {
        if (bgProb[c](i)<bgProbMin)
          bgProbImg->imageData[i]=bgProbMin;// improves visualisation
        else
          bgProbImg->imageData[i]=(bgProb[c](i)-bgProbMin)*255.0/(bgProbMax-bgProbMin);
      }
      // convert to Ros image and publish
      cv_bridge::CvImage bgProbImgRos;
      bgProbImgRos.encoding = "mono8";
      bgProbImgRos.image = bgProbImg;
      sensor_msgs::ImagePtr imagePtr=bgProbImgRos.toImageMsg();
      backgroundsPub[c].publish(imagePtr);
      //cvShowImage(dynBGProb,bgProbImg);
    }
#else

    cvCvtColor(src_vec[c], cvt_vec[c], TO_INT_FMT);
    img2vec(cvt_vec[c], img_vec[c]);
    
    bgModel[c].getBackground(img_vec[c], bg_vec[c]);
    cam[c].computeBGProbDiff(img_vec[c], bg_vec[c], sumPixel[c], sum_g[c]);

#endif

    if (frameCounter->count(c)>INIT_FIRST_FRAMES)
    {
      ROS_INFO_STREAM("detect");
      accompany_uva_msg::HumanDetections humanDetections;   
      humanDetections = findPerson(c, src_vec, img_vec, bg_vec, sum_g, logSumPixelFGProb, sumPixel);
      humanDetectionsPub.publish(humanDetections);
      markerArrayPub.publish(msgToMarkerArray.toMarkerArray(humanDetections,frame.child_frame_id)); // publish visualisation

    }
    else
      ROS_INFO_STREAM("wait for other frames");

    if (visualize) cvWaitKey(waitTime);

#if USE_DYNAMIC_BACKGROUND
    if (visualize)
      cvReleaseImage(&bgProbImg);
    cvReleaseImage(&smooth);
#endif
    
    //cvReleaseImage(&src_vec[c]);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
  }
}

void publishTF()
{
  while (running)
  {
    frame.header.stamp=ros::Time::now();
    transformBroadcasterPtr->sendTransform(frame);
    usleep(50*1000); // sleep 50ms for +-20Hz thread
  }
}

void timerCallback(const ros::TimerEvent& timerEvent)
{
  // publish transform
  frame.header.stamp=ros::Time::now();
  transformBroadcasterPtr->sendTransform(frame);
}

int main(int argc, char **argv)
{
  string params_file;

  // handling arguments
  po::options_description optionsDescription(
      "Human Detection main function\n"
      "Available remappings:\n"
      "  humanDetections:=<humanDetections-topic>\n"
      "\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("params_file,p", po::value<string>(&params_file)->required(),"filename of params.xml")
    ("num_persons,n", po::value<unsigned int>(&NUM_PERSONS)->default_value(4))
    ("visualize,v","visualize detection\n")
    ("threads,t", po::value<unsigned int>(&nrThreads)->default_value(4),"number of threads used");
    ("save_all,a", po::value<string>(&save_all)->default_value(""),"save all data\n");

  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    po::notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }

  visualize=variablesMap.count("visualize"); // are we visualizing the frames and detections?

  if (!save_all.empty())
  {
    sprintf(data_file,"%s/data.xml",save_all.c_str());
  }

  string path, bgmodel_file, prior_file, frame_file;
  boost::filesystem::path p(params_file);
  path = p.parent_path().string().c_str();
  bgmodel_file = path + "/" + "bgmodel.xml";
  prior_file = path + "/" + "prior.txt";
  frame_file = path + "/" + "frame.dat";

  
  ROS_INFO_STREAM("loading '"<<bgmodel_file.c_str()<<"'");
  // load coordinate frame of camera
  if (!load_msg(frame,frame_file))
  {
    std::cerr<<"Failed to load the coordinate frame from file '"<<frame_file<<"'. Use program 'create_frame' to create the file."<<endl;
    exit(1);
  }

  ROS_INFO_STREAM("loading '"<<bgmodel_file.c_str()<<"'");
  // Initialize localization module
#if USE_DYNAMIC_BACKGROUND
#else
  getBackground(bgmodel_file.c_str(), bgModel); // load background model
#endif
  ROS_INFO_STREAM("loading '"<<params_file.c_str()<<"'");
  loadCalibrations(params_file.c_str()); // load calibration
  ROS_INFO_STREAM("loading '"<<prior_file.c_str()<<"'");
  loadHull(prior_file.c_str(), priorHull);

#if USE_DYNAMIC_BACKGROUND
#else
  assert_eq(bgModel.size(), CAM_NUM);
  assert_eq(cam.size(), bgModel.size());
#endif
  ROS_INFO_STREAM("generate Scan locations"); 
  genScanLocations(priorHull, scanres, scanLocations);

  logLocPrior.set_size(scanLocations.size());
  logLocPrior.fill(-log(scanLocations.size()));
  ROS_INFO_STREAM("loading done\n"); 

  // ROS nodes, subscribers and publishers
  ros::init(argc, argv, "camera_localization");
  ros::NodeHandle n;
  std::string resolved_humanDetections=n.resolveName("humanDetections");
  cout<<"publish to detection topic: "<<resolved_humanDetections<<endl;

  tf::TransformBroadcaster transformBroadcaster;
  transformBroadcasterPtr=&transformBroadcaster;
  
  image_transport::ImageTransport it(n);
  humanDetectionsPub = n.advertise<accompany_uva_msg::HumanDetections>(resolved_humanDetections, 10);
  markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);

  frameCounter=new FrameCounter(cam.size());
  backgroundsPub=new image_transport::Publisher[cam.size()];
  detectionsPub=new image_transport::Publisher[cam.size()];
  for (unsigned int i=0;i<cam.size();i++)
  {
    // subscribe to images
    string topicName=cam[i].name+"/image_raw";
    ROS_INFO_STREAM("subscribe to '"<<topicName<<"'");
    image_transport::CameraSubscriber sub = it.subscribeCamera(topicName,1,imageCallback);
    
    // init publishers for background images
    string backgroundTopicName=cam[i].name+"/background/image_raw";
    ROS_INFO_STREAM("publish to '"<<backgroundTopicName<<"'");
    backgroundsPub[i]=it.advertise(backgroundTopicName,1);

    // init publishers for detection images
    string detectTopicName=cam[i].name+"/detect/image_raw";
    ROS_INFO_STREAM("publish to '"<<detectTopicName<<"'");
    detectionsPub[i]=it.advertise(detectTopicName,1);

    // init images
    src_vec[i]=NULL;
  }
  
  boost::thread publishTFThread(publishTF);
  //ros::Timer timer=n.createTimer(ros::Duration(0.01),timerCallback); // use thread to avoid waiting on processing frames

  ROS_INFO_STREAM("wait for frames");
  ros::spin();
  
  running=false;
  publishTFThread.join();

  for (unsigned int c=0;c<cam.size();c++)
    if (src_vec[c]!=NULL)
      cvReleaseImage(&src_vec[c]);

  return 0;
}
