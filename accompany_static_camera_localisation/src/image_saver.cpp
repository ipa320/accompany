#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

int g_count = 0;
int nrImage;
string encoding = "bgr8";
string imgExt = ".jpg";

class ImageSaver
{
public:
  ImageSaver(string path)
  {
    this->path=path;
    backgroundList.open((path+"/"+"background_list.txt").c_str());
    cout<<"constructed ImageSaver to path: "<<path<<endl;
  }

  ~ImageSaver()
  {
    backgroundList.close();
  }

  void callback(const sensor_msgs::ImageConstPtr& image)
  {
    // ros::Time now=ros::Time::now();
    ros::Time acquisition_time = image->header.stamp; // use acquisition time instead of now()
    cv_bridge::CvImageConstPtr cv_ptr=cv_bridge::toCvShare(image,encoding);
    stringstream ss;
    ss<<path<<"/"<<setfill('0')<<setw(12)<<acquisition_time.sec<<"."
      <<setfill('0')<<setw(9)<<acquisition_time.nsec<<imgExt;
    string filename=ss.str();
    cout<<"saving "<<filename<<endl;
    cv::imwrite(filename,cv_ptr->image);
    backgroundList<<filename<<endl;

    if (nrImage>0)
      if (++g_count>nrImage-1)
        ros::shutdown();
  }

private:
  string path;
  ofstream backgroundList;
};


int main(int argc, char** argv)
{
  vector<string> paths;
  vector<string> topics;
  vector<ImageSaver*> imageSavers;
  unsigned int nrTopics;

  // handling arguments
  po::options_description optionsDescription(
      "Subscribe to ros topics and write images to disk. Example usage:\n\n"
      "  image_saver -t topic1 -p /home/ros/temp1 -t topic2 -p /home/ros/temp2\n\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h", "produce help message")
    ("nr,n", po::value<int>(&nrImage)->default_value(10),"total number of images to save, <=0 means unlimited")
    ("topic,t", po::value<vector<string> >(&topics)->required(),"topic to read images from")
    ("depth_image,d", "specify if it is the depth image")
    ("path,p", po::value<vector<string> >(&paths),"global path to save images to");
  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    po::notify(variablesMap);
  }
  catch (const exception& e)
  {
    cerr << "--------------------" << endl;
    cerr << "- " << e.what() << endl;
    cerr << "--------------------" << endl;
    cerr << optionsDescription << endl;
    return 1;
  }

  cout<<"number of images to save: "<<nrImage<<endl;

  if (variablesMap.count("depth_image"))
  {
    encoding = "mono16";
    imgExt = ".png";
  }
  cout << "encoding: " << encoding << endl;

  if (paths.size()==0) // set default when non given
    paths.push_back("./");

  nrTopics=topics.size();
  if (paths.size()<nrTopics)
    nrTopics=paths.size();

  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  vector<image_transport::Subscriber> subs;
  for (unsigned i=0;i<nrTopics;i++)
  {
    ImageSaver *imageSaver=new ImageSaver(paths[i]);
    cout<<"subscribe to topic: "<<topics[i]<<endl;
    subs.push_back(it.subscribe(topics[i], 1, &ImageSaver::callback,imageSaver));
    imageSavers.push_back(imageSaver);
  }

  ros::spin();
  for (unsigned i=0;i<nrTopics;i++)
    delete imageSavers[i]; // close files

  return 0;
}
