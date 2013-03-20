#include <AppearanceExtractor.h>

using namespace std;

/**
 * Resize pixels and unclaim
 * @param image use image size
 */
void PixelsClaimed::clear(IplImage *image)
{
  int size=image->width*image->height;
  pixels.resize(size);
  clear();
}

/**
 * Claim a pixel by setting to a value other than '0'
 * @param i the pixel to claim
 */
unsigned char& PixelsClaimed::operator[](int i)
{
  return pixels[i];
}

void PixelsClaimed::clear()
{
  for (unsigned i=0;i<pixels.size();i++)
    pixels[i]=0;
}
  

/**
 * Compute combined appearance of all detections over the images of camera c
 * @param c the camera to use
 * @param cam each cameras
 * @param existing each detection
 * @param scanLocations locations of detections
 * @param masks the masks for each camera, for each position
 * @param images current images of each cameras
 * @param bgProb log background probablities for each image
 * @returns 
 */
void AppearanceExtractor::computeAppearance(int c,
                                            const vector<CamCalib>& cam,
                                            const vector<unsigned>& existing,
                                            const vector<WorldPoint>& scanLocations,
                                            const vector<vector<vector<scanline_t> > > masks,
                                            vector<IplImage *> images,
                                            const vector<vnl_vector<FLOAT> >& bgProb)
{
  IplImage *image=images[c];
  //vnl_vector<FLOAT> bg=bgProb[c];
  pixelsClaimed.clear(image);
    
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
          unsigned ind=p*3;
          image->imageData[ind+0]=255-order[person]*50;
          image->imageData[ind+1]=255-order[person]*50;
          image->imageData[ind+2]=255-order[person]*50;
        }
        pixelsClaimed[p]=1; // claim pixel
      }
    }
  }
}

/**
 * Compute combined appearance of all detections over current images of each cameras
 * @param cam each cameras
 * @param existing each detection
 * @param scanLocations locations of detections
 * @param masks the masks for each camera, for each position
 * @param images current images of each cameras
 * @param bgProb log background probablities for each image
 * @returns 
 */
void AppearanceExtractor::computeAppearances(const vector<CamCalib>& cam,
                                             const vector<unsigned>& existing,
                                             const vector<WorldPoint>& scanLocations,
                                             const vector<vector<vector<scanline_t> > > masks,
                                             vector<IplImage *> images,
                                             const vector<vnl_vector<FLOAT> >& bgProb)
{
  cout<<"computeAppearances"<<endl;
  for (unsigned c=0;c<cam.size();c++)
    computeAppearance(c,cam,existing,scanLocations,masks,images,bgProb);
}


/**
 * Order the detections ascending by order of distance to the camera
 * @param cam the camera
 * @param existing the detections
 * @param scanLocations locations of detections
 * @returns vector of indices of existing
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
