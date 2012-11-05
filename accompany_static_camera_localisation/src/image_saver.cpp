#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
//#include <camera_calibration_parsers/parse.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

sensor_msgs::CvBridge g_bridge;
boost::format g_format;
int g_count = 0;
int nrImage;
string path;

ofstream backgroundList;

void callback(const sensor_msgs::ImageConstPtr& image)
{
  if (g_bridge.fromImage(*image, "bgr8"))
  {
    IplImage *image = g_bridge.toIpl();
    if (image)
    { 
        string filename = (g_format % g_count % "jpg").str();
        cvSaveImage((path+"/"+filename).c_str(), image);
	backgroundList<<(path+"/"+filename).c_str()<<endl;
        ROS_INFO("Saved image %s", filename.c_str());
	if (++g_count>nrImage-1)
	  ros::shutdown();
    }
    else
    {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", image->encoding.c_str());
} 

int main(int argc, char** argv)
{
  // handling arguments
  po::options_description optionsDescription(
      "Image saver\n"
      "Available remappings:\n"
      "  image:=<image-topic>\n"
      "\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h", "produce help message")
    ("nr,n", po::value<int>(&nrImage)->default_value(10),"number of images to save")
    ("path,p", po::value<string>(&path)->default_value("./"),"path to save images to");
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
  if (variablesMap.count("help")) 
  {
    cout<<optionsDescription<<endl;
    return 1;
  }

  cout<<"number of images to save: "<<nrImage<<endl;
  cout<<"path to save images to: '"<<path<<"'"<<endl;

  backgroundList.open((path+"/"+"background_list.txt").c_str());

  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  string resolved_image=nh.resolveName("image");
  cout<<"subscribe to image topic: "<<resolved_image<<endl;

  g_format.parse("%04i.%s");
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe(resolved_image, 1, &callback);

  ros::spin();
  backgroundList.close();
  return 0;
}
