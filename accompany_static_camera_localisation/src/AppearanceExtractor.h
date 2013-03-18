#ifndef AppearanceExtractor_INCLUDED
#define AppearanceExtractor_INCLUDED

#include <vector>
#include <vnl/vnl_vector.h>
#include <iostream>
using namespace std;

// deal with occulsions by foreground, allow detections that are closer to the camera to claim the pixels
class PixelsClaimed
{
 public:
  PixelsClaimed()
  {
  }

  void clear(IplImage *image)
  {
    int size=image->width*image->height;
    pixels.resize(size);
    clear();
  }

  void clear()
  {
    for (unsigned i=0;i<pixels.size();i++)
      pixels[i]=0;
  }
  
  unsigned char& operator[](int i)
  {
    return pixels[i];
  }
  
 private:
  std::vector<unsigned char> pixels;
};

// extract the appearance of each detection
class AppearanceExtractor
{
 
 public:

  // compute appearance of all detections for camera 'c'
  void computeAppearance(int c,
                         const std::vector<CamCalib>& cam,
                         const std::vector<unsigned>& existing,
                         const vector<WorldPoint>& scanLocations,
                         const std::vector<std::vector<std::vector<scanline_t> > > masks,
                         std::vector<IplImage *> images,
                         const std::vector<vnl_vector<FLOAT> >& bgProb)
  {
    IplImage *image=images[c];
    //vnl_vector<FLOAT> bg=bgProb[c];
    pixelsClaimed.clear(image);
    
    cout<<"computeAppearance: "<<c<<endl;
    std::vector<int> order=orderDetections(cam[c],existing,scanLocations);
    cout<<"order ";
    for (std::vector<int>::iterator it=order.begin();it!=order.end();it++)
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

  // compute appearance of all detections over all cameras
  void computeAppearances(const std::vector<CamCalib>& cam,
                          const std::vector<unsigned>& existing,
                          const vector<WorldPoint>& scanLocations,
                          const std::vector<std::vector<std::vector<scanline_t> > > masks,
                          std::vector<IplImage *> images,
                          const std::vector<vnl_vector<FLOAT> >& bgProb)
  {
    cout<<"computeAppearances"<<endl;
    for (signed c=0;c<cam.size();c++)
      computeAppearance(c,cam,existing,scanLocations,masks,images,bgProb);
  }

 private:

  PixelsClaimed pixelsClaimed;

  // return indices that order 'existing' on distance to the camera
  std::vector<int> orderDetections(const CamCalib& cam,
                                   const std::vector<unsigned>& existing,
                                   const vector<WorldPoint>& scanLocations)
  {
    cout<<"orderDetections"<<endl;
    // create indices
    std::vector<int> order(existing.size());
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

  // compute square distance of camera position to world point
  double squareDistance(const CamCalib& cam,
                        const WorldPoint& point)
  {
    
    cv::Mat camPos=cam.model.tvec;
    double dx=camPos.at<double>(0)-point.x;
    double dy=camPos.at<double>(1)-point.y;
    double dz=camPos.at<double>(2)-point.z;
    cout<<"squareDistance ("
        <<camPos.at<double>(0)<<" "
        <<camPos.at<double>(1)<<" "
        <<camPos.at<double>(2)<<") ("
        <<point.x<<" "
        <<point.y<<" "
        <<point.z<<")"
        <<endl;
    return dx*dx+dy*dy+dz*dz;
  }

};

#endif
