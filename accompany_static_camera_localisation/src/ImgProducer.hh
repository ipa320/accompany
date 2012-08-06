#ifndef IMG_PRODUCER_HH
#define IMG_PRODUCER_HH

#include <vector>
#include <string>
#include <opencv/highgui.h>
#include <opencv/cv.h>

class ImgProducer {
    std::vector<IplImage *> img;
    CvCapture *capture;
    std::vector< std::vector<std::string> > filelist;
    unsigned nextFrame, numFrames, numCam;
    enum sourceType_t { FILELIST, CAPTURE } sourcetype;
  public:
    ImgProducer(const char *filename);

    std::vector<IplImage *>getFrame();
    const std::vector<std::string> &getNextSource() const;
    const std::vector<std::string> &getPrevSource() const;
    void forward(unsigned frames);
    void backward(unsigned frames);
    unsigned getNumCam() const { return numCam; }
    unsigned getNumFrames() const { return numFrames; }
};

#endif  // IMG_PRODUCER_HH
