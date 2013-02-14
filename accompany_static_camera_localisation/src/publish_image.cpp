

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace boost::program_options;

variables_map variablesMap;
int abWidth,abHeight;
double scWidth,scHeight;

cv::Mat resize;

vector< vector<string> > readImages(vector<string> files)
{
  vector< vector<string> > ret(files.size());
  
  for (unsigned int i=0;i<files.size();i++)
  {
    string line;
    ifstream myfile(files[i].c_str());
    if (myfile.is_open())
    {
      cout<<"reading '"<<files[i]<<"'"<<endl;
      while ( myfile.good() )
      {
        getline (myfile,line);
        if (line.length()>3) // if somthing like a path to file was read
        {
          cout << line << endl;
          ret[i].push_back(line);
        }
      }
      myfile.close();
    }
    else
    {
      cerr<<"unable to open file '"<<files[i]<<"'"<<endl;
      exit(1);
    }
  }
  return ret;
}

int main(int argc,char **argv)
{
  double frequency;
  bool loop;
  vector<string> imageFiles,topic_outs;
  
  // handling arguments
  options_description optionsDescription("Reads images from disk and publishes them. Example usage:\n\n"
                                         "  publish_image -i imageFile1.txt -o topic1 -i imageFile2.txt -o topic2\n\n"
                                         "all options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("frequency,f", value<double>(&frequency)->default_value(1.0),"frequency for publishing images")
    ("loop,l", value<bool>(&loop)->default_value(false),"loop back to beginning when all images are published")
    ("imagefile,i", value<vector<string> >(&imageFiles)->required(),"files with global path to an image in each line")
    ("topic_out,o", value<vector<string> >(&topic_outs)->required(),"names of output topic");

  variables_map variablesMap;
  try
  {
    store(parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }

  // get number of topics
  cout<<topic_outs.size()<<endl;
  cout<<imageFiles.size()<<endl;
  unsigned int nrTopics=topic_outs.size();
  if (imageFiles.size()<nrTopics)
    nrTopics=imageFiles.size();

  cout<<"publishing to:"<<endl;
  for (unsigned int i=0;i<nrTopics;i++)
    cout<<"  "<<topic_outs[i]<<endl;

  // read images
  vector< vector<string> > images=readImages(imageFiles);

  // get min number of images over all files
  unsigned int nrImages=0;
  for (unsigned int i=0;i<images.size();i++)
    if (images[i].size()<nrImages || i==0)
      nrImages=images[i].size();
  cout<<"smallest number of images in all files: "<<nrImages<<endl;

  ros::init(argc, argv, "publish_image");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  vector<image_transport::Publisher> pubs;
  for (unsigned int i=0;i<nrTopics;i++)
    pubs.push_back(it.advertise(topic_outs[i],1));

  unsigned int i=0;
  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    // for all topics
    for (unsigned int j=0;j<nrTopics;j++)
    {
      // read image
      cv::Mat image = cv::imread(images[j][i]);
      if(!image.empty()) // check for invalid input
      {
        // publish image
        cv_bridge::CvImage rosImage;
        rosImage.encoding = "bgr8";
        rosImage.image = image;
        // republish image
        pubs[j].publish(rosImage.toImageMsg());
      }
      else
         cerr<<"unable to open image '"<<images[j][i]<<"'"<<endl;
    }

    if ((++i)>=nrImages) // end detected
    {
      if (loop)
      {
        cout<<"loop to beginning"<<endl;
        i=0;
      }
      else
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
