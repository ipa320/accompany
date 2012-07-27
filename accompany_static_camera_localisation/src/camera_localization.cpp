
#include <ros/ros.h>
#include <accompany_human_tracker/HumanLocations.h>
#include <accompany_static_camera_localisation/HumanLocationsParticle.h>
#include <accompany_static_camera_localisation/HumanLocationsParticles.h>

#include <ctime>
#include <cstdlib>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/version.hpp>
#if BOOST_VERSION < 103500
#include <boost/thread/detail/lock.hpp>
#endif
#include <boost/thread/xtime.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <stdio.h>
#include "cmn/FastMath.hh"
#include "cmn/random.hh"
#include "tools/utils.hh"
#include "tools/string.hh"
#include "Helpers.hh"
#include "Background.hh"
#include <data/XmlFile.hh>
#include "ImgProducer.hh"
#include "CamCalib.hh"
#include "fstream"

using namespace std;
using namespace cv;
//#include <boost/program_options.hpp>
//namespace po = boost::program_options;

bool PRODUCER_BUILDS_BACKGROUND  = 1;
bool SYNCHRONISED_PRODUCER = 1;
unsigned MIN_TRACK_LEN = 20;
unsigned MAX_TRACK_JMP = 1000;
unsigned MAX_TRACK_AGE = 8;


// struct Image_t {
// };

struct Buffer_t {
     bool consumed;
     unsigned index;

     vector<string> sourceName;
     vector<IplImage *>img;
     vector< vnl_vector<FLOAT> > imgVec, bgVec;
     vector< vnl_vector<FLOAT> > logSumPixelBGProb;
     vector<FLOAT> imgProb;

};

Buffer_t *current = NULL, buffers[2];
int currentIndex = 0;
boost::mutex imgProtect, mutexA, mutexB;
bool running = false;

extern unsigned w, w2;
vector<Background> bgModel(0,0);
vector<FLOAT> logNumPrior;
vnl_vector<FLOAT> logLocPrior;
vector< WorldPoint > priorHull;
vector< vnl_vector<FLOAT> > logSumPixelFGProb;
vector< vector< vector<scanline_t> > > masks; // camera, id, lines
vector<WorldPoint> scanLocations;
int waitTime = 20;

ImgProducer *producer;

const char *win[] = {
     "fg_1","fg_2","fg_3","fg_4","fg_5","fg_6","fg_7","fg_8","fg_9","fg_10",
     "fg_11","fg_12","fg_13","fg_14","fg_15","fg_16","fg_17","fg_18","fg_19","fg_20"
};
const char *bgwin[] = {
     "bg_1","bg_2","bg_3","bg_4","bg_5","bg_6","bg_7","bg_8","bg_9","bg_10",
     "bg_11","bg_12","bg_13","bg_14","bg_15","bg_16","bg_17","bg_18","bg_19","bg_20"
};

CvScalar trackCol [] = {
     CV_RGB( 0, 0, 255 ),             // Blue    1
     CV_RGB( 0, 255, 0 ),             // Green   2
     CV_RGB( 255, 0, 0 ),             // Red     3
     CV_RGB( 255, 255, 0 ),           // Cyan    4
     CV_RGB( 255, 0, 255 ),           // magenta 5
     CV_RGB( 0, 255, 255 ),           // Yellow  6
     CV_RGB( 0, 128, 255 ),           // Orange  7
     // CV_RGB( 255, 255, 255 ),         // White   8
     CV_RGB( 128, 0, 0 ),             // Blue    9
     CV_RGB( 0, 128, 0 ),             // Green   10
     CV_RGB( 0, 0, 128 ),             // Red     11
     CV_RGB( 128, 128, 0 ),           // Cyan    12
     CV_RGB( 128, 0, 128 ),           // magenta 13
     CV_RGB( 0, 128, 128 ),           // Yellow  14
     CV_RGB( 0, 64, 128 ),            // Orange  15
     CV_RGB( 128, 128, 128 ),         // Grey    16
     CV_RGB( 128, 0, 0 ),             //
     CV_RGB( 0, 128, 0 ),
     CV_RGB( 0, 0, 128 )
};

struct track_t {
     vector<WorldPoint> positions;
     vector<unsigned> imgID;

     FLOAT sumR, sumG, sumB;
     FLOAT sumR2, sumG2, sumB2;
     FLOAT sumPix;
};
vector<track_t> tracks;

void buildMasks()
{
     masks = vector< vector< vector<scanline_t> > > (cam.size(), vector< vector<scanline_t> >(scanLocations.size()));

     for (unsigned n=0; n!=cam.size(); ++n)
          for (unsigned i=0; i!=scanLocations.size(); ++i) {
               vector<CvPoint> tpl;
               cam[n].genTemplate(scanLocations[i],tpl);
               getMask(tpl,masks[n][i]);
          }
}

FLOAT logMaskProb(const vnl_vector<FLOAT> &logSumPixelFGProb,
                  const vnl_vector<FLOAT> &logSumPixelBGProb, FLOAT bgSum, const vector<scanline_t> &mask)
{
     // cerr << "logSumPixelFGProb.size()=" << logSumPixelFGProb.size() << ", logSumPixelBGProb.size()="
     //      << logSumPixelBGProb.size() << ", mask.size()=" << mask.size() << endl;
     // cerr << "logSumPixelFGProb.size()=" << logSumPixelFGProb.size() << endl;
     FLOAT res = bgSum;
     for (vector<scanline_t>::const_iterator i=mask.begin(); i!=mask.end(); ++i) {
          unsigned offset = i->line * (width+1) + 1; // if start=0, offset-1 == 0
          //cout << "offset=" << offset << ", start=" << i->start << ", end=" << i->end << ", size=" << logSumPixelBGProb.size() << endl;
          res -= (logSumPixelBGProb(offset + i->end) - logSumPixelBGProb(offset + i->start - 1));
          // res += (logSumPixelFGProb(offset + i->end) - logSumPixelFGProb(offset + i->start - 1))
          //      - (logSumPixelBGProb(offset + i->end) - logSumPixelBGProb(offset + i->start - 1));
     }

     return res;
}

FLOAT logMaskProbDiff(const vnl_vector<FLOAT> &logSumPixelFGProb,
                      const vnl_vector<FLOAT> &logSumPixelBGProb, const vector<scanline_t> &mask)
{
     FLOAT res = 0;
     for (vector<scanline_t>::const_iterator i=mask.begin(); i!=mask.end(); ++i) {
          unsigned offset = i->line * (width+1) + 1; // if start=0, offset-1 == 0
          res -= (logSumPixelBGProb(offset + i->end) - logSumPixelBGProb(offset + i->start - 1));
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
 **/
bool overlap(const vector<unsigned> &existing, unsigned pos)
{
     WorldPoint &p2 = scanLocations[pos];
     for (unsigned i=0; i!=existing.size(); ++i) {
          WorldPoint &pt = scanLocations[existing[i]];
          if (p2.x >= pt.x - w2 && p2.x <= pt.x + w2 &&
              p2.y >= pt.y - w2 && p2.y <= pt.y + w2)
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
              vector< vector<scanline_t> > &existingMask,
              const vector< vnl_vector<FLOAT> > &logSumPixelFGProb,
              const vector< vnl_vector<FLOAT> > &logSumPixelBGProb,
              const vector<FLOAT> &logNumPrior,
//              const vnl_vector<FLOAT> &logLocPrior, // remove?
              vector< vnl_vector<FLOAT> > &logPosProb,
              vector< FLOAT > &marginal,
              const vector<FLOAT> &lSum)
{
     unsigned
          bestIdx = 0;
     FLOAT
          bestLogProb = -INFINITY,
          marginalLogProb = -INFINITY;

     logPosProb.push_back(vnl_vector<FLOAT>(scanLocations.size(),-INFINITY));
     vnl_vector<FLOAT>
          &lpp = logPosProb.back();

     static vector<FLOAT>
          logPosProbCache(scanLocations.size(),-INFINITY);

     // cout << "marginal=[ ";
     // for (unsigned i=0; i!=marginal.size(); ++i)
     //      cout << marginal[i] << " ";
     // cout << "]" << endl;
     if (existing.size()) 
     {
          unsigned &hereIdx = existing.back();
          for (unsigned c=0; c!=cam.size(); ++c)
               mergeMasks(existingMask[c],masks[c][hereIdx]);

          FLOAT
               logNP = logNumPrior[existing.size()+1];

          for (unsigned p=0; p!=scanLocations.size(); ++p)
          {
          
               if (!overlap(existing, p)  && logPosProbCache[p]>0) 
               {
                    // cout << "Considering position " << p << ": (" << scanLocations[p].x << "," << scanLocations[p].y << ")" <<endl;
                    // if (logLocPrior(p) < -5000)  --> Not part of scanLocations anymore
                    //      continue;
                    lpp(p) = logLocPrior(p) + logNP;
                    for (unsigned c=0; c!=cam.size(); ++c) {
                         vector<scanline_t>
                              mask = existingMask[c];
                         mergeMasks(mask,masks[c][p]);
                         lpp(p) += logMaskProb(logSumPixelFGProb[c], logSumPixelBGProb[c],
                                               lSum[c], mask);
                    }
                    marginalLogProb = log_sum_exp(marginalLogProb,lpp(p));
                    if (lpp(p) > bestLogProb) {
                         bestLogProb = lpp(p);
                         bestIdx = p;
                    }
               }
           }
        } 
     else {
          FLOAT
               logNP = logNumPrior[1];
               
               
////////          std::ofstream f2;
////////          f2.open("lpp.txt",std::fstream::app);
          
          for (unsigned p=0; p!=scanLocations.size(); ++p) 
          {
               // if (logLocPrior(p) < -5000) --> Not part of scanLocations anymore
               //      continue;

               logPosProbCache[p] = logLocPrior(p);
               for (unsigned c=0; c!=cam.size(); ++c) {
                    // cout << "Camera " << c << endl;
                    logPosProbCache[p] +=
                         logMaskProb(logSumPixelFGProb[c], logSumPixelBGProb[c], lSum[c], masks[c][p]);  // already includes lSum
               }
               lpp(p) = logPosProbCache[p] + logNP;

//////////               f2 << lpp(p) << ",";
               logPosProbCache[p] -= logNumPrior[0];
               for (unsigned c=0; c!=cam.size(); ++c) 
               {
                    logPosProbCache[p] -= lSum[c]; // hack -> if pos, increases lik; if neg, doesn't
               }
               marginalLogProb = log_sum_exp(marginalLogProb,lpp(p));
               if (lpp(p) > bestLogProb) {
                    bestLogProb = lpp(p);
                    bestIdx = p;
               }
          }
//////////          f2 << endl;
//////////          f2.close();

     }
     // cout << "bestIdx=" << bestIdx << ", bestLogProb=" << bestLogProb << endl;
//      if (marginalLogProb > marginal.back()) {
     if (bestLogProb > marginal.back()) { // TODO
          existing.push_back(bestIdx);
          // marginal.push_back(marginalLogProb);
          marginal.push_back(bestLogProb);

//          if (existing.size() > 2)// TODO knowing the number of persons in the scene
//              return;
          
          scanRest(existing, existingMask, logSumPixelFGProb, logSumPixelBGProb,
                   logNumPrior, /*logLocPrior, */logPosProb, marginal, lSum);
     }
}

CvPoint cvPoint(unsigned pos)
{
     return cvPoint(pos%width,pos/width);
}

/**
 * \brief Distance between two points in the ground plane
 * \param p1
 * \param p2
 * \return
 *
 * that is, ignore the z component.
 *
 * 2010/11/08: GWENN - First version
 *
 **/
FLOAT dist(const WorldPoint &p1, const WorldPoint &p2)
{
     double
          x1 = p1.x,
          y1 = p1.y,
          x2 = p2.x,
          y2 = p2.y,
          d1 = x1-x2,
          d2 = y1-y2;
     return sqrt(d1*d1 + d2*d2);
}

void plotHull(IplImage *img, vector<WorldPoint> &hull, unsigned c)
{
     hull.push_back(hull.front());
     for (unsigned i=1; i<hull.size(); ++i)
          cvLine(img, cam[c].project(hull[i-1]),cam[c].project(hull[i]),CV_RGB(255,0,0),2);
}

void plotTrack(vector<IplImage *> &img,const track_t &t, CvScalar colour, unsigned w)
{
     for (unsigned c=0; c!=img.size(); ++c) {
          for (unsigned j=1; j<t.positions.size(); ++j) {
               cvCircle (img[c],cam[c].project(t.positions[j-1]),3,colour,w);
               cvLine(img[c],cam[c].project(t.positions[j-1]),cam[c].project(t.positions[j]),colour,w);
          }
          cvCircle (img[c],cam[c].project(t.positions.back()),3,colour,w);
          // plotTemplate2(img[c],cam[c].project(t.positions.back()),persHeight,camHeight,colour);
     }
}

void updateTracks(unsigned imgNum, const vector<unsigned> &positions)
{
     vnl_matrix<float>
          distances(tracks.size(),positions.size());
     for (unsigned i=0; i!=tracks.size(); ++i)
          for (unsigned j=0; j!=positions.size(); ++j)
               distances(i,j) = dist(tracks[i].positions.back(),scanLocations[positions[j]]);
     vector<bool>
          trackAssigned(tracks.size(),false),
          obsAssigned(positions.size(),false);
     bool
          foundAssignment = true;
     while(foundAssignment) {
          foundAssignment = false;

          float
               bestDist = MAX_TRACK_JMP;
          unsigned
               bestTrack = 9999999, bestObs=88888888;
          for (unsigned i=0; i!=tracks.size(); ++i)
               for (unsigned j=0; j!=positions.size(); ++j)
                    if (distances(i,j) < bestDist && !trackAssigned[i] && !obsAssigned[j]) {
                         foundAssignment = true;
                         bestDist = distances(i,j);
                         bestTrack = i;
                         bestObs = j;
                    }
          if (foundAssignment) {
               trackAssigned[bestTrack] = true;
               obsAssigned[bestObs] = true;
               tracks[bestTrack].positions.push_back(scanLocations[positions[bestObs]]);
               tracks[bestTrack].imgID.push_back(imgNum);
          }
     }

     for (unsigned i=0; i!=obsAssigned.size(); ++i) {
          if (!obsAssigned[i]) {
               track_t t;
               t.positions.push_back(scanLocations[positions[i]]);
               t.imgID.push_back(imgNum);
               tracks.push_back(t);
          }
     }

     vector<track_t>::iterator i=tracks.begin();
     while (i!=tracks.end()) {
          if (imgNum - i->imgID.back() > MAX_TRACK_AGE) {
               // Person left FOV
               tracks.erase(i);
               i = tracks.begin();
          } else
               ++i;
     }
}

accompany_human_tracker::HumanLocations findPerson(unsigned imgNum,
                vector<IplImage *>src,
                const vector< vnl_vector<FLOAT> > &imgVec,
                vector< vnl_vector<FLOAT> > &bgVec,
                const vector<FLOAT> logBGProb,
                const vector< vnl_vector<FLOAT> > &logSumPixelFGProb,
                const vector< vnl_vector<FLOAT> > &logSumPixelBGProb)
{
     // stic();

     vector<FLOAT>
          marginal;
     FLOAT
          lNone = logNumPrior[0];

     for (unsigned c=0; c!=cam.size(); ++c)
          lNone += logBGProb[c];
     FLOAT
          lSum = -INFINITY;

     vector<unsigned>
          existing;
     vector< vnl_vector<FLOAT> >
          logPosProb;
     vector< vector<scanline_t> >
          mask(cam.size());


     marginal.push_back(lNone);
     logPosProb.push_back(vnl_vector<FLOAT>());
     // cout << "Marginal[0]=" << marginal.back() << endl;
     scanRest(existing, mask, logSumPixelFGProb, logSumPixelBGProb, logNumPrior,
              /*logLocPrior, */logPosProb, marginal, logBGProb);

     // write detection results
////////     ofstream f1;
////////     f1.open("detectionResults.txt",std::fstream::app);
////////     for (unsigned i=0; i!=existing.size(); ++i)
////////     {
//////////          cout << existing[i] << endl;
////////          f1 << " " << scanLocations[existing[i]];
////////     }
////////     f1 << endl;
////////     f1.close();

     
     // REPORTING LOCATIONS
     cout << "locations found are" << endl;
     accompany_human_tracker::HumanLocations humanLocations;
     geometry_msgs::Vector3 v;
     for (unsigned i=0; i!=existing.size(); ++i)
     {
//          cout << existing[i] << endl;
          WorldPoint wp = scanLocations[existing[i]];
          cout << " " << scanLocations[existing[i]];
          v.x=wp.x;
          v.y=wp.y;
          v.z=0;
          humanLocations.locations.push_back(v);
     }
     cout << endl << "===========" << endl;
     
     for (unsigned i=0; i!=marginal.size(); ++i)
          lSum = log_sum_exp(lSum,marginal[i]);
     unsigned
          mlnp = 0;             // most likely number of people
     FLOAT
          mlprob = -INFINITY;
     for (unsigned i=0; i!=marginal.size(); ++i) {
          // cout << "p(n=" << i << ") = " << exp(marginal[i]-lSum)
          //      << " (log = " << marginal[i] << ")"
          //      << endl;
          if (marginal[i] > mlprob) {
               mlnp = i;
               mlprob = marginal[i];
          }
     }

     updateTracks(imgNum, existing);

     // for (unsigned i=0; i!=tracks.size(); ++i) {
     //      if (tracks[i].imgID.size() > MIN_TRACK_LEN)
     //           if (imgNum- tracks[i].imgID.back() < 2)
     //                // plotTrack(src, tracks[i], CV_RGB(0,255,0),1);
     //                plotTrack(src, tracks[i], trackCol[i],2);
     // }

     // cout << " ";
     // stoc();
     // cout << endl;

     // for (unsigned i=2; i<bgVec.size(); i+=3)
     //      bgVec(i) = 255;

     static int number = 0;
     number++;
     static char buffer[1024];

     /* Visualize tracks */
     for (unsigned c=0; c!=cam.size(); ++c) {
          IplImage
               *bg = vec2img((/*imgVec[c]-*/bgVec[c]).apply(fabs))
               ,*cvt = cvCreateImage(cvGetSize(bg),IPL_DEPTH_8U,3)
          ;
          cvCvtColor(bg,cvt,TO_IMG_FMT);

          plotHull(src[c],priorHull,c);
          plotHull(cvt,priorHull,c);
          // For comparison:
          for (unsigned i=0; i!=existing.size(); ++i) {
               vector<CvPoint>
                    tplt;
               cam[c].genTemplate(scanLocations[existing[i]],tplt);
               plotTemplate(cvt,tplt,CV_RGB(255,255,255));
               plotTemplate(src[c],tplt,CV_RGB(0,255,255));
               cvCircle(src[c],cam[c].project(scanLocations[existing[i]]),1,CV_RGB(255,255,0),2);
          }

//////          cvShowImage(bgwin[c], cvt); // TODO
          cvReleaseImage(&bg);
          cvReleaseImage(&cvt);
          cvShowImage(win[c],src[c]); //<<--- // TODO
          snprintf(buffer, sizeof(buffer), "movie/%04d.jpg",number); //"movie/%08d-%d.jpg",number,c);
////////           cvSaveImage(buffer, src[c]);
     }
     /* End of Visualization */

     int k = cvWaitKey(waitTime);
     switch (k) {
     case 1048608:         // space
     case 537919520:       // Space at uni ...?
          waitTime = 20-waitTime; break;
     case 1113939:         // ->
     case 537984851:
          producer->forward(10); break;
     case 1376083:         // ctrl ->
     case 1179475:         // shift ->
     case 538246995:
          producer->forward(100); break;
     case 1113937:         // <-
     case 537984995:
          producer->backward(10); break;
     case 1179473:         // shift <-
     case 1376081:         // ctrl <-
     case 538181731:
          producer->backward(100);
          break;
     case -1:              // nothin'
          break;
     default:
          // cout << key << endl;
          break;
     }
     
     return humanLocations;
}

void initStaticProbs() {
     unsigned
          maxN = 50;
     logNumPrior = vector<FLOAT>(maxN, -log((FLOAT)maxN));

     logNumPrior.push_back(-INFINITY); // p(#=LAST)

     unsigned wp1 = width+1;
     logSumPixelFGProb.resize(cam.size());
     logSumPixelFGProb[0] = vnl_vector<FLOAT>((wp1)*height);
     FLOAT U = -3.0 * log(256.0);
     for (unsigned i=0; i!=logSumPixelFGProb[0].size(); ++i) {
     //      if (i%(width+1) == 0)
     //           logSumPixelFGProb(i) = 0;
     //      else
     //           logSumPixelFGProb(i) = logSumPixelFGProb(i-1) + U;
          logSumPixelFGProb[0](i) = (i%(wp1)) * U;
     }
     for (unsigned c=1; c!=cam.size(); ++c)
          logSumPixelFGProb[c] = logSumPixelFGProb[0];
}

int main(int argc,char **argv)
{
//   options
  bool particles=false;
  int nrParticles = 10;
//  // boost parameters parsing
//  po::options_description optionsDescription("Allowed options");
//  optionsDescription.add_options()
//    ("help", "produce help message")
//    ("particles,p","publish particles and do not publish human locations directly")
//    ("nrparticles,n", po::value<int>(&nrParticles)->default_value(10),"number of particles to sample (implies -p)")
//    ;
//  po::variables_map variablesMap;
//  po::store(po::parse_command_line(argc, argv, optionsDescription), variablesMap);
//  po::notify(variablesMap);
//  
//  if (variablesMap.count("help"))
//  {
//    cout<<optionsDescription<<endl;
//    return 1;
//  }
//  if (variablesMap.count("particles") || variablesMap.count("nrparticles"))
//    particles=true;

  // init ros
  ros::init(argc, argv, "CameraLocalisation");

  // read files

  // create publishers and subscribers
  ros::NodeHandle n;
  ros::Publisher humanLocationsPub=n.advertise<accompany_human_tracker::HumanLocations>("/humanLocations",10);
  ros::Publisher humanLocationsParticlesPub=n.advertise<accompany_static_camera_localisation::HumanLocationsParticles>("/humanLocationsParticles",10);

  // generate dummy data
  int max=100;
  int count=0;
  int direction=1;


  srand(time(0));// initialize random number generator




     if (argc != 4) {
          cout << "Usage: threadtest <bg train|model> <calib> <prior>" << endl;
          return 1;
     }

     unsigned CAM_NUM = 1;

     // INITIALIZATION
     getBackground(argv[1], bgModel);
     loadCalibrations(argv[2]);
     loadWorldPriorHull(argv[3], priorHull);
     assert_eq(bgModel.size(), CAM_NUM);
     assert_eq(cam.size(), bgModel.size());
     genScanLocations(priorHull,scanres, scanLocations);
     
     logLocPrior.set_size(scanLocations.size());
     logLocPrior.fill(-log(scanLocations.size()));
        
     // load the first image to get image size //TODO
     IplImage* testImage = loadImage("test_frames/frame0000.jpg"); // TODO release after use
     vector<IplImage *> image = vector<IplImage *>(CAM_NUM);
     image[0] = testImage;

     width    = image[0]->width;
     height   = image[0]->height;
     depth    = image[0]->depth;
     channels = image[0]->nChannels;
     halfresX = width/2;
     halfresY = height/2;

     initStaticProbs();
     buildMasks();

     vector<IplImage*> src, cvt(CAM_NUM);
     unsigned cnt = 0;

     
     src = image;

     for (unsigned c=0; c!=cam.size(); ++c) {
         if (!src[c]) exit(0);
          cvt[c] = cvCreateImage(cvGetSize(src[c]),IPL_DEPTH_8U,3);
     }
     vector< vnl_vector<FLOAT> >
          img(CAM_NUM),bg(CAM_NUM);
     vector< vnl_vector<FLOAT> >
          sumPixel(cam.size());
     vector<FLOAT>
          sum(cam.size());
          
     
  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    accompany_human_tracker::HumanLocations humanLocations;
  
    for (unsigned c=0; c!=cam.size(); ++c)
    {
        cvCvtColor(src[c],cvt[c],TO_INT_FMT);

        img2vec(cvt[c],img[c]);
        bgModel[c].getBackground(img[c],bg[c]);

        /*
    // publish human locations
    accompany_human_tracker::HumanLocations humanLocations;
    geometry_msgs::Vector3 v;
    v.x=10+((direction<0)*max+count*direction)*0.1;
    v.y=1;
    v.z=0;
    humanLocations.locations.push_back(v);
    v.x=3;
    v.y=10+((direction<0)*max+count*direction)*0.2;
    v.z=0;
    humanLocations.locations.push_back(v);
    v.x=5+count*0.15;
    v.y=5+count*0.15;
    v.z=0;
    humanLocations.locations.push_back(v);
        */

        if (PRODUCER_BUILDS_BACKGROUND)
            cam[c].computeBGProbDiff(img[c], bg[c], sumPixel[c],sum[c]);
    }
    humanLocations = findPerson(cnt++, src, img, bg, sum, logSumPixelFGProb, sumPixel);
    
    if (++count>=max)
    {
      count=0;
      direction*=-1;
    }
    if (!particles)
    humanLocationsPub.publish(humanLocations);

    // publish human locations particles
    if (particles)
    {
      accompany_static_camera_localisation::HumanLocationsParticles humanLocationsParticles;
      for (int i=0;i<nrParticles;i++)
      {
        accompany_static_camera_localisation::HumanLocationsParticle humanLocationsParticle;
        int numberOfLocations=(rand()%humanLocations.locations.size())+1;
        for (int l=0;l<numberOfLocations;l++)
        {
          int randomLoc=(rand()%humanLocations.locations.size());// random location index
          geometry_msgs::Vector3 v=humanLocations.locations[randomLoc];// random location vector
          humanLocationsParticle.locations.push_back(v);// add location to particle
        }
        humanLocationsParticle.weight=rand()/((double)RAND_MAX);// set random weight
        humanLocationsParticles.particles.push_back(humanLocationsParticle);
      }
      humanLocationsParticlesPub.publish(humanLocationsParticles);
    }
    
    // get new image //TODO
    // release image here // TODO
    IplImage* testImage = loadImage("test_frames/frame0000.jpg"); //TODO
    vector<IplImage *> image = vector<IplImage *>(CAM_NUM);
    image[0] = testImage;
    src=image;

    for (unsigned c=0; c!=cam.size(); ++c) {
        if (!src[c]) 
        {
            cout << "camera " << c << "is empty, exit" << endl;
            exit(0);
        }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
