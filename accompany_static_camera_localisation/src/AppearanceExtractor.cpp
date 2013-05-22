#include <AppearanceExtractor.h>

#include <LogProbOp.h>

using namespace std;

/**
 * Constructor, resize pixels and unclaim
 * @param image use image size
 */
PixelsClaimed::PixelsClaimed(IplImage *image)
{
  pixels.resize(image->width*image->height);
  clear();
}

/**
 * Claim a pixel by setting to a value other than '0', A pixel can be claimed only ones.
 * @param i the pixel to claim
 */
unsigned char& PixelsClaimed::operator[](int i)
{
  return pixels[i];
}

/**
 * Unclaim all pixels
 */
void PixelsClaimed::clear()
{
  for (unsigned i=0;i<pixels.size();i++)
    pixels[i]=0;
}

CvScalar getcolor(int p,double weight)
{
  CvScalar scalar;
  p++;// start at 1
  scalar.val[0]=(p&1)?(255*weight):0; // define color using bitmask op p
  scalar.val[1]=(p&2)?(255*weight):0;
  scalar.val[2]=(p&4)?(255*weight):0;
  return scalar;
}

/**
 * Compute combined appearance of all detections over the last image of camera c
 * @param c the camera to use
 * @param cam each cameras
 * @param existing each detection
 * @param scanLocations locations of detections
 * @param masks the masks for each camera, for each position
 * @param images current images of each cameras
 * @param bgProb log background probablities for each image
 * @returns color histogram
 */
vector<HISTOGRAM > AppearanceExtractor::computeAppearances(int c,
                                                          const vector<CamCalib>& cam,
                                                          const vector<unsigned>& existing,
                                                          const vector<WorldPoint>& scanLocations,
                                                          const vector<vector<vector<scanline_t> > > masks,
                                                          vector<IplImage *> images,
                                                          const vector<vnl_vector<FLOAT> >& bgProb)
{
  vector<HISTOGRAM > histograms(existing.size());
  IplImage *image=images[c];
  vnl_vector<FLOAT> bg=bgProb[c];
  PixelsClaimed pixelsClaimed(image);
    
  cout<<"computeAppearance: "<<c<<endl;
  vector<int> order=orderDetections(cam[c],existing,scanLocations);
  cout<<"order ";
  for (vector<int>::iterator it=order.begin();it!=order.end();it++)
    cout<<*it<<" ";
  cout<<endl;
    
  for (unsigned person=0;person<existing.size();person++)
  {      
    vector<scanline_t> mask=masks[c][existing[order[person]]];
    for (vector<scanline_t>::const_iterator it = mask.begin(); it != mask.end();++it) // each scanline
    {
      unsigned offset = it->line*image->width;
      unsigned start=offset+it->start;
      unsigned end=offset+it->end;
      for (unsigned p=start;p<end;p++) // each pixel
      {
        if (pixelsClaimed[p]==0) // if unclaimed
        {
          // weight= P(B=0|P)= P(P|B=0) / (P(P|B=0) + P(P|B=1))
          //                 = (1/pow(256,3)) / (1/(pow(256,3)) + exp(bg(p)));
          HIST_TYPE_WEIGHT weight=exp(-3*log(256)-LogProbOp<HIST_TYPE_WEIGHT>::add(-3*log(256),bg(p)));
          unsigned ind=p*3;
          histograms[order[person]].add(((unsigned char *)(image->imageData))+ind,weight);
          // visualize
          CvScalar color=getcolor(order[person],weight);
          image->imageData[ind+0]=color.val[0];
          image->imageData[ind+1]=color.val[1];
          image->imageData[ind+2]=color.val[2];
        }
        pixelsClaimed[p]=person+1; // claim pixel
      }
    }
  }
  return histograms;
}

/**
 * Compute combined appearance of all detections over the last image of each camera
 * @param cam each cameras
 * @param existing each detection
 * @param scanLocations locations of detections
 * @param masks the masks for each camera, for each position
 * @param images current images of each cameras
 * @param bgProb log background probablities for each image
 * @returns color histogram
 */
vector<HISTOGRAM > AppearanceExtractor::computeAppearances(const vector<CamCalib>& cam,
                                                                  const vector<unsigned>& existing,
                                                                  const vector<WorldPoint>& scanLocations,
                                                                  const vector<vector<vector<scanline_t> > > masks,
                                                                  vector<IplImage *> images,
                                                                  const vector<vnl_vector<FLOAT> >& bgProb)
{
  cout<<"computeAppearances"<<endl;
  vector<HISTOGRAM > histograms(existing.size());
  for (unsigned c=0;c<cam.size();c++)
  {
    vector<HISTOGRAM > histsOnCam=computeAppearances(c,cam,existing,scanLocations,masks,images,bgProb);
    for (unsigned h=0;h<histograms.size();h++)
      histograms[h]+=histsOnCam[h];
  }
  return histograms;
}

/**
 * Order the detections ascending by order of distance to the camera
 * @param cam the camera
 * @param existing the detections
 * @param scanLocations locations of detections
 * @returns vector of indices of existing that determine the order
 */
vector<int> AppearanceExtractor::orderDetections(const CamCalib& cam,
                                                 const vector<unsigned>& existing,
                                                 const vector<WorldPoint>& scanLocations)
{
  cout<<"orderDetections"<<endl;
  // create indices
  vector<int> order(existing.size());
  for (unsigned i=0;i<existing.size();i++)
    order[i]=i;
  // bubble sort indices
  bool sort=true;
  while (sort)
  {
    sort=false;
    for (unsigned i=1;i<existing.size();i++)
    {
      if (squareDistance(cam,scanLocations[existing[order[i]]])<
          squareDistance(cam,scanLocations[existing[order[i-1]]]))
      {
        unsigned temp=order[i-1];
        order[i-1]=order[i];
        order[i]=temp;
        sort=true;
      }
    }
  }
  return order;
}

/**
 * Compute square distance from point to camera
 * @param cam the camera
 * @param point the point
 */
double AppearanceExtractor::squareDistance(const CamCalib& cam,
                                           const WorldPoint& point)
{
  cv::Mat camPos=cam.model.getImage2WorldTransMatrix();
  double dx=camPos.at<double>(0)-point.x;
  double dy=camPos.at<double>(1)-point.y;
  double dz=camPos.at<double>(2)-point.z;
  return dx*dx+dy*dy+dz*dz;
}
