#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
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
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#if BOOST_VERSION < 103500
#include <boost/thread/detail/lock.hpp>
#endif

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>

#include <accompany_uva_utils/uva_utils.h>
#include <accompany_uva_msg/HumanLocations.h>
#include <accompany_uva_msg/HumanLocationsParticle.h>
#include <accompany_uva_msg/HumanLocationsParticles.h>

#include "cmn/FastMath.hh"
#include "cmn/random.hh"
#include "tools/utils.hh"
#include "tools/string.hh"
#include "Helpers.hh"
#include "Background.hh"
#include <data/XmlFile.hh>
#include "ImgProducer.hh"
#include "CamCalib.hh"

namespace po = boost::program_options;
using namespace std;
using namespace cv;

unsigned MIN_TRACK_LEN = 20;
unsigned MAX_TRACK_JMP = 1000;
unsigned MAX_TRACK_AGE = 8;

extern unsigned w2;
vector<Background> bgModel(0, 0);
vector<FLOAT> logNumPrior;
vnl_vector<FLOAT> logLocPrior;
vector<WorldPoint> priorHull;
vector<vnl_vector<FLOAT> > logSumPixelFGProb;
vector<vector<vector<scanline_t> > > masks; // camera, id, lines
vector<WorldPoint> scanLocations;

geometry_msgs::TransformStamped frame;
tf::TransformBroadcaster *transformBroadcasterPtr;
ros::Publisher humanLocationsPub, humanLocationsParticlesPub;
sensor_msgs::CvBridge bridge;
unsigned int CAM_NUM = 1;
int NUM_PERSONS;
bool HAS_INIT = false;
vector<IplImage*> cvt_vec(CAM_NUM);
vector<vnl_vector<FLOAT> > img_vec(CAM_NUM), bg_vec(CAM_NUM);
vector<IplImage *> src_vec(CAM_NUM);

// particle options
bool particles = false;
int nrParticles = 10;
int max = 100;
int count = 0;
int direction = 1;

ImgProducer *producer;

int waitTime = 30;
const char* win = "foreground";
const char* bgwin = "background";

string saveImagesPath;
string imagePostfix;
int visualize;

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
  for (vector<scanline_t>::const_iterator i = mask.begin(); i != mask.end();
      ++i)
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
  for (vector<scanline_t>::const_iterator i = mask.begin(); i != mask.end();
      ++i)
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
  FLOAT bestLogProb = -INFINITY, marginalLogProb = -INFINITY;

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
        marginalLogProb = log_sum_exp(marginalLogProb, lpp(p));
        if (lpp(p) > bestLogProb)
        {
          bestLogProb = lpp(p);
          bestIdx = p;
        }
      }
    }
  }
  else
  {
    FLOAT logNP = logNumPrior[1];

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
      marginalLogProb = log_sum_exp(marginalLogProb, lpp(p));
      if (lpp(p) > bestLogProb)
      {
        bestLogProb = lpp(p);
        bestIdx = p;
      }
    }

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

accompany_uva_msg::HumanLocations findPerson(unsigned imgNum,
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

  // report locations
  cout << "locations found are" << endl;
  accompany_uva_msg::HumanLocations humanLocations;
  
  geometry_msgs::Vector3Stamped v;
  std_msgs::Header header;
  header.stamp=ros::Time::now();
  header.frame_id=frame.child_frame_id;
  v.header=header;// set current time and frame name to the vector
  for (unsigned i = 0; i != existing.size(); ++i)
  {
    WorldPoint wp = scanLocations[existing[i]];
    cout << " " << scanLocations[existing[i]];
    v.vector.x = wp.x/1000; // millimeters to meters
    v.vector.y = wp.y/1000;
    v.vector.z = 0;
    humanLocations.locations.push_back(v);
  }
  cout << endl << "===========" << endl;

  for (unsigned i = 0; i != marginal.size(); ++i)
    lSum = log_sum_exp(lSum, marginal[i]);
  unsigned mlnp = 0; // most likely number of people
  FLOAT mlprob = -INFINITY;
  for (unsigned i = 0; i != marginal.size(); ++i)
  {
    // cout << "p(n=" << i << ") = " << exp(marginal[i]-lSum)
    //      << " (log = " << marginal[i] << ")"
    //      << endl;
    if (marginal[i] > mlprob)
    {
      mlnp = i;
      mlprob = marginal[i];
    }
  }

  static int number = 0;
  number++;
//  static char buffer[1024];

  /* Visualize tracks */
  for (unsigned c = 0; c != cam.size(); ++c)
  {
    IplImage *bg = vec2img((/*imgVec[c]-*/bgVec[c]).apply(fabs));
    IplImage *cvt = cvCreateImage(cvGetSize(bg), IPL_DEPTH_8U, 3);
    cvCvtColor(bg, cvt, TO_IMG_FMT);

    plotHull(src[c], priorHull, c);
    plotHull(cvt, priorHull, c);
    // For comparison:
    for (unsigned i = 0; i != existing.size(); ++i)
    {
      vector<CvPoint> tplt;
      cam[c].genTemplate(scanLocations[existing[i]], tplt);
      plotTemplate(cvt, tplt, CV_RGB(255,255,255));
      plotTemplate(src[c], tplt, CV_RGB(0,255,255));
      cvCircle(src[c], cam[c].project(scanLocations[existing[i]]), 1,
          CV_RGB(255,255,0), 2);
    }

    //		cvShowImage(bgwin, cvt);
    cvReleaseImage(&bg);
    cvReleaseImage(&cvt);
    if (visualize) cvShowImage(win, src[c]);
    
    if (saveImagesPath!="")
    {
      ros::Time begin = ros::Time::now();
      stringstream ss;
      ss<<saveImagesPath<<"/"<<setfill('0')<<setw(12)<<begin.sec
        <<setfill('0')<<setw(9)<<begin.nsec<<imagePostfix<<".png";
      cvSaveImage(ss.str().c_str(),src[c]);
    }

    if (visualize) cvWaitKey(waitTime);
    //		snprintf(buffer, sizeof(buffer), "movie/%04d.jpg",number); //"movie/%08d-%d.jpg",number,c);
    //		cvSaveImage(buffer, src[c]);
  }
  /* End of Visualization */

  return humanLocations;
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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  vector<vnl_vector<FLOAT> > sumPixel(cam.size());
  vector<FLOAT> sum_g(cam.size());

  try
  {
    // load the first image to get image size
    IplImage* oriImage = bridge.imgMsgToCv(msg, "bgr8");
    src_vec[0] = cvCloneImage(oriImage);

    if (!HAS_INIT)
    {
      width = src_vec[0]->width;
      height = src_vec[0]->height;
      depth = src_vec[0]->depth;
      channels = src_vec[0]->nChannels;
      halfresX = width / 2;
      halfresY = height / 2;

      initStaticProbs();
      buildMasks();

      for (unsigned c = 0; c != cam.size(); ++c)
      {
        if (!src_vec[c])
        {
          ROS_ERROR("empty image frame");
        }
        cvt_vec[c] = cvCreateImage(cvGetSize(src_vec[c]), IPL_DEPTH_8U, 3);
      }
      HAS_INIT = true;
    }

    accompany_uva_msg::HumanLocations humanLocations;
    for (unsigned c = 0; c != cam.size(); ++c)
    {
      cvCvtColor(src_vec[c], cvt_vec[c], TO_INT_FMT);
      img2vec(cvt_vec[c], img_vec[c]);
      bgModel[c].getBackground(img_vec[c], bg_vec[c]);
      cam[c].computeBGProbDiff(img_vec[c], bg_vec[c], sumPixel[c], sum_g[c]);
    }

    humanLocations = findPerson(0, src_vec, img_vec, bg_vec, sum_g,
        logSumPixelFGProb, sumPixel);
    cvReleaseImage(&src_vec[0]);

    if (!particles)
      humanLocationsPub.publish(humanLocations);

    // publish human locations particles
    //        if (particles)
    //        {
    //            accompany_uva_msg::HumanLocationsParticles humanLocationsParticles;
    //            for (int i=0;i<nrParticles;i++)
    //            {
    //            accompany_uva_msg::HumanLocationsParticle humanLocationsParticle;
    //            int numberOfLocations=(rand()%humanLocations.locations.size())+1;
    //            for (int l=0;l<numberOfLocations;l++)
    //            {
    //              int randomLoc=(rand()%humanLocations.locations.size());// random location index
    //              geometry_msgs::Vector3Stamped v=humanLocations.locations[randomLoc];// random location vector
    //              humanLocationsParticle.locations.push_back(v);// add location to particle
    //            }
    //            humanLocationsParticle.weight=rand()/((double)RAND_MAX);// set random weight
    //            humanLocationsParticles.particles.push_back(humanLocationsParticle);
    //          }
    //          humanLocationsParticlesPub.publish(humanLocationsParticles);
    //        }
    //        cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
    //        cvDestroyWindow("view");

  }

  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void timerCallback(const ros::TimerEvent& timerEvent)
{
  // publish transform
  frame.header.stamp=ros::Time::now();
  transformBroadcasterPtr->sendTransform(frame);
  // also publish transform with time in the future so that tf always has a 'current' transform
  frame.header.stamp=ros::Time::now()+ros::Duration(2);
  transformBroadcasterPtr->sendTransform(frame);
}

int main(int argc, char **argv)
{
  string path, bgmodel_file, params_file, prior_file, intrinsic_file, extrinsic_file, frame_file;
  
  // handling arguments
  po::options_description optionsDescription(
      "Human Detection main function\n"
      "Available remappings:\n"
      "  image:=<image-topic>\n"
      "  humanLocations:=<humanLocations-topic>\n"
      "\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("path_param,p", po::value<string>(&path)->required(),"path where you put all files, including bgmodel.xml,"
     "param.xml, prior.txt, camera_intrinsic.xml, camera_extrinsic.xml\n")
    ("num_persons,n", po::value<int>(&NUM_PERSONS)->default_value(-1))
    ("saveImagePath,s", po::value<string>(&saveImagesPath)->default_value(""),"path to save images to\n")
    ("imagePostfix,i", po::value<string>(&imagePostfix)->default_value(""),"postfix of image name\n")
    ("visualize,v","visualize detection\n");

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

  bgmodel_file = path + "/" + "bgmodel.xml";
  params_file = path + "/" + "params.xml";
  prior_file = path + "/" + "prior.txt";
  intrinsic_file = path + "/" + "camera_intrinsic.xml";
  extrinsic_file = path + "/" + "camera_extrinsic.xml";
  frame_file = path + "/" + "frame.dat";

  cout<<"loading '"<<frame_file<<"'"<<endl;
  // load coordinate frame of camera
  if (!load_msg(frame,frame_file))
  {
    std::cerr<<"Failed to load the coordinate frame from file '"<<frame_file<<"'. Use program 'create_frame' to create the file."<<endl;
    exit(1);
  }

  cout<<"loading '"<<bgmodel_file<<"'"<<endl;
  // Initialize localization module
  getBackground(bgmodel_file.c_str(), bgModel);
  cout<<"loading '"<<intrinsic_file<<"'"<<endl;
  cout<<"loading '"<<extrinsic_file<<"'"<<endl;
  loadCalibrations(params_file.c_str(), intrinsic_file.c_str(),
      extrinsic_file.c_str());
  cout<<"loading '"<<prior_file<<"'"<<endl;
  loadWorldPriorHull(prior_file.c_str(), priorHull);
  assert_eq(bgModel.size(), CAM_NUM);
  assert_eq(cam.size(), bgModel.size());
  genScanLocations(priorHull, scanres, scanLocations);

  logLocPrior.set_size(scanLocations.size());
  logLocPrior.fill(-log(scanLocations.size()));
  cout<<"loading done"<<endl;

  // ROS nodes, subscribers and publishers
  ros::init(argc, argv, "camera_localization");
  ros::NodeHandle n;
  std::string resolved_image=n.resolveName("image");
  std::string resolved_humanLocations=n.resolveName("humanLocations");
  cout<<"subscribe to image topic: "<<resolved_image<<endl;
  cout<<"publish to location topic: "<<resolved_humanLocations<<endl;

  tf::TransformBroadcaster transformBroadcaster;
  transformBroadcasterPtr=&transformBroadcaster;
  ros::Timer timer=n.createTimer(ros::Duration(0.1),timerCallback);
  image_transport::ImageTransport it(n);
  humanLocationsPub = n.advertise<accompany_uva_msg::HumanLocations>(resolved_humanLocations, 10);
  //humanLocationsParticlesPub=n.advertise<accompany_uva_msg::HumanLocationsParticles>("/humanLocationsParticles",10);
  image_transport::Subscriber sub = it.subscribe(resolved_image, 1,imageCallback);
  ros::spin();

  return 0;
}
