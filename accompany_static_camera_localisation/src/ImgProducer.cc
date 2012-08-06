#include "ImgProducer.hh"
#include "Helpers.hh"
#include <tools/string.hh>
#include <iostream>

using namespace std;

ImgProducer::ImgProducer(const char *filename) :
    nextFrame(0)
{
  if (matchesnc(filename, "*.avi") || matchesnc(filename, "rtsp://*"))
  {
    sourcetype = CAPTURE;
    capture = cvCreateFileCapture(filename);
    if (!capture)
      goto err;
    img = vector<IplImage *>(1);
    numFrames = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
    numCam = 1;
  }
  else
  {
    sourcetype = FILELIST;
    listImages(filename, filelist);
    if (filelist.size() == 0)
      goto err;
    numCam = filelist[0].size();
    img = vector<IplImage *>(numCam);
    numFrames = filelist.size();
    cerr << "Number of files: " << numFrames << endl;
    cerr << "Number of cam: " << numCam << endl;
  }

  return;
  err: cerr << "cannot open " << filename << endl;
  exit(1);
}

const vector<string> &ImgProducer::getNextSource() const
{
  static vector<string> junk(1, "Not a file");
  if (sourcetype == FILELIST && nextFrame < filelist.size())
    return filelist[nextFrame];
  else
    return junk;
}

const vector<string> &ImgProducer::getPrevSource() const
{
  static vector<string> junk(1, "Not a file");
  if (sourcetype == FILELIST && nextFrame - 1 < filelist.size())
    return filelist[nextFrame - 1];
  else
    return junk;
}

vector<IplImage *> ImgProducer::getFrame()
{
  switch (sourcetype)
  {
    case CAPTURE:
      img[0] = cvQueryFrame(capture);
      return img;
    case FILELIST:
      for (unsigned c = 0; c != img.size(); ++c)
      {
        if (img[c])
          cvReleaseImage(&img[c]);
        if (nextFrame == filelist.size())
          img[c] = NULL;
        else
        {
          // cout << "loading img " << nextFrame << "," << c << ":"
          //      << filelist[nextFrame][c].c_str() << endl;
          img[c] = loadImage(filelist[nextFrame][c].c_str());
          if (img[c] == NULL)
            cerr << "Cannot load img " << c << ": "
                << filelist[nextFrame][c].c_str() << endl;
        }
      }
      nextFrame++;
      return img;
    default:
      cerr << "Case not implemented" << endl;
      for (unsigned c = 0; c != img.size(); ++c)
        img[c] = NULL;
      return img;
  }
}

void ImgProducer::forward(unsigned frames)
{
  switch (sourcetype)
  {
    case CAPTURE:
      nextFrame = cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES);
      cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, nextFrame + frames);
      break;
    case FILELIST:
      if (nextFrame + frames < filelist.size())
        nextFrame += frames;
      else
        nextFrame = filelist.size() - 1;
      break;
    default:
      cerr << "Case not implemented" << endl;
      break;
  }

}

void ImgProducer::backward(unsigned frames)
{
  switch (sourcetype)
  {
    case CAPTURE:
      nextFrame = cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES);
      if (nextFrame >= frames)
        cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES,
            nextFrame - frames);
      else
        cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, 0);
      break;
    case FILELIST:
      if (nextFrame >= frames)
        nextFrame -= frames;
      else
        nextFrame = 0;
      break;
    default:
      cerr << "Case not implemented" << endl;
      break;
  }

}
