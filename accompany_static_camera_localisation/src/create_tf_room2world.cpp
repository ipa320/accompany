#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <accompany_uva_utils/uva_utils.h>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

Scalar BLUE = CV_RGB(0,0,255), GREEN = CV_RGB(0,255,0), RED = CV_RGB(255,0,0);
vector<Scalar> COLORS;
string text = "OXY";
float scale = 1;
vector<Point> locations;
Mat img;

void showScaledImg()
{
  Mat scaled;
  Point pnt(10, 10);
  Point pnt2(-5, 15);
  Size dst_sz(round(scale * img.cols), round(scale * img.rows));
  resize(img, scaled, dst_sz, 0, 0);
  for (unsigned int i = 0; i < locations.size(); ++i)
  {
    circle(scaled, locations[i] * scale, 2, COLORS[i], 1);
    rectangle(scaled, (locations[i] - pnt) * scale,
        (locations[i] + pnt) * scale, COLORS[i], 2 * scale);
    putText(scaled, text.substr(i, 1), (locations[i] - pnt2) * scale,
        FONT_HERSHEY_SIMPLEX, 0.5 * scale, COLORS[i], 2 * scale);
  }
  imshow("image", scaled);
}

void mouseHandler(int event, int x, int y, int flags, void *param)
{

  Point p = Point(x, y);
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      p *= 1 / scale;
      cout << "Left button down at " << p.x << "," << p.y << endl;
      if (locations.size() >= 3)
        cout << "You have clicked 3 points already. Press 'q' to continue or 'z' to redo last point" << endl;
      else
      {
        locations.push_back(p);
        if (locations.size() == 2)
        {
          Point p0 = locations[0];
          Point p1 = locations[1];
          Point v1 = p1 - p0;
          Point v2;
          v2.x = v1.y;
          v2.y = v1.x * (-1);
          Point p2 = p0 + v2;
          locations.push_back(p2);
          cout << "You have clicked 3 points already. Press 'q' to continue or 'z' to redo last point" << endl;
        }
      }
      showScaledImg();
      break;
    }
  }
}

void printUsage()
{
  cout << "Options:" << endl;
  cout << "  " << "LEFT Mouse   " << "select points" << endl;
  cout << "  " << "+" << "      " << "zoom in" << endl;
  cout << "  " << "-" << "      " << "zoom out" << endl;
  cout << "  " << "z" << "      " << "step back" << endl;
  cout << "  " << "q" << "  " << "save&quit" << endl;
}

int main(int argc, char **argv)
{
  string imFile, mapParamFile;

  // handling arguments
  po::options_description optionsDescription(
      "Find tf from room coordinates to world coordinates\nAllowed options\n");
  optionsDescription.add_options()("map,m",
      po::value<string>(&imFile)->required(), "the world map\n")("param,p",
      po::value<string>(&mapParamFile)->required(), "parameters of the map\n");

  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),
        variablesMap);
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

  float resolution;
  vector<float> origin;
  cv::FileStorage fsParam(mapParamFile, cv::FileStorage::READ);
  FileNode fn = fsParam["origin"];
  fsParam["resolution"] >> resolution;
  fn >> origin;
  cout << origin[0] << origin[1] << origin[2] << endl;
  cout << resolution << endl;

  img = imread(imFile);
  namedWindow("image");
  setMouseCallback("image", mouseHandler, NULL);
  imshow("image", img);

  COLORS.push_back(BLUE);
  COLORS.push_back(RED);
  COLORS.push_back(GREEN);

  int key = 0;
  while (true)
  {
    if (locations.size() == 0)
    {
      printUsage();
    }
    
    key = waitKey(0);
    
    if ((char) key == 'q' && locations.size() == 3) // exit
      break;

    switch ((char) key)
    {
      case '-':
        cout << "zoom out" << endl;
        if (scale > .1)
          scale -= .1;
        showScaledImg();
        break;

      case '+':
        cout << "zoom in" << endl;
        if (scale < 4)
          scale += .1;
        showScaledImg();
        break;

      case 'z':
        cout << "step back" << endl;
        if (locations.size())
          locations.pop_back();
        showScaledImg();
        break;

      default:
        printUsage();
        break;
    }
  }

  Mat src_points = Mat::zeros(Size(2, 3), CV_32F);
  for (unsigned int i = 0; i < locations.size(); ++i)
  {
    src_points.at<float>(i, 0) = locations[i].x * resolution;
    src_points.at<float>(i, 1) = (img.rows - locations[i].y) * resolution;
  }

  cout << "src_points are: " << src_points << endl;
  cout << "-------------------------------------" << endl;

  Mat dst_points = Mat::zeros(Size(2, 3), CV_32F);
  cout << "input the world coordinates of 'O', separate with SPACE" << endl;
  cin >> dst_points.at<float>(0, 0) >> dst_points.at<float>(0, 1);
  cout << "input the world coordinates of 'X', separate with SPACE" << endl;
  cin >> dst_points.at<float>(1, 0) >> dst_points.at<float>(1, 1);
  cout << "input the world coordinates of 'Y', separate with SPACE" << endl;
  cin >> dst_points.at<float>(2, 0) >> dst_points.at<float>(2, 1);
  cout << "dst_points are: " << dst_points << endl;
  cout << "-------------------------------------" << endl;

  Mat tform;
  tform = getAffineTransform(src_points, dst_points);
  cout << "src_points: " << endl << src_points << endl;
  cout << "dst_points: " << endl << dst_points << endl;
  cout << "transform matrix: " << endl << tform << endl;
  cout << "How it works: " << endl
      << "dst_points = transform matrix * [src_points'; 1,1,1]" << endl;


  char *filename="frame.dat";
  cout<<"create some frame and write to file '"<<filename<<"'"<<endl;
  geometry_msgs::TransformStamped transformStamped;
  tf::Transform transform = tf::Transform(
                  btMatrix3x3(tform.at<float>(0,0),tform.at<float>(0,1),0,// rotation matrix
                              tform.at<float>(1,0),tform.at<float>(1,1),0,
                              0,0,1), 
                  btVector3(tform.at<float>(0,2),tform.at<float>(1,2),0));// translation vector
  tf::StampedTransform stampedTransform=tf::StampedTransform(transform,     // the transform
                                                             ros::Time(0),  // time, not used here
                                                             "/map",        // parent coordinate frame
                                                             "/overhead1"); // child coordinate frame
  tf::transformStampedTFToMsg(stampedTransform,transformStamped);
  save_msg(transformStamped,filename); // write to file
 

  cout<<"read from file '"<<filename<<"' and print, just a test:"<<endl;
  geometry_msgs::TransformStamped transformStamped2;
  load_msg(transformStamped2,filename);
  cout << transformStamped2;
  
  waitKey(0);
  return 0;
}
