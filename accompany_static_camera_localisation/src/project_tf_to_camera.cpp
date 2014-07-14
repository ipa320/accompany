#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <sstream>

namespace po = boost::program_options;
using namespace std;

bool view_on;
bool save_on = false;
string save_file;
float time_shift;
cv::Mat col_vals; // buffer of skeleton points (TimexNumx2)
cv::Mat timestamp; // buffer of time stamp

// void loadData(cv::FileStorage fs, cv::Mat& col_names, cv::Mat& col_vals)
// {
//   openFile(fs,"r");
//   fs["col_names"] >> col_names;
//   fs["col_vals"] >> col_vals;
//   // fs.release();
// }

// write skeleton points to xml/yaml

void writeData(std::vector<std::string> frame_ids_, cv::Mat col_vals, cv::Mat timestamp)
{

  cv::FileStorage fs(save_file, cv::FileStorage::WRITE);

  if (!fs.isOpened())
  {
    std::cout << "Error: cannot open file " << save_file << std::endl;
    exit(1);
  }

  fs << "timestamp" << timestamp; // timestamp to file
  for (unsigned int i = 0; i < frame_ids_.size() ; i++)
  {
    std::string frame_id = frame_ids_[i];
    frame_id.erase(0,1); // '/' cannot be written into yml file, so remove
    fs << frame_id << col_vals.col(i).reshape(1); // joint points to file
  }

}

// Class FrameDrawer transform tf to cameras and visualize in a new image

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  cv_bridge::CvImagePtr cv_ptr;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_; // list of frame_id

public:
  FrameDrawer(const std::vector<std::string>& frame_ids, std::string image_topic)
    : it_(nh_), frame_ids_(frame_ids)
  {
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    if (view_on)
    {
      ROS_INFO("subscribed to camera %s",image_topic.c_str());
      pub_ = it_.advertise("image_out", 1);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    vector<cv::Point2d> pts;
    ros::Duration time_compensate(time_shift); //TODO:

    ros::Time acquisition_time = info_msg->header.stamp + time_compensate; // TODO

    BOOST_FOREACH(const std::string& frame_id, frame_ids_)
    {
      tf::StampedTransform transform;
      try
      {
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv_rect,uv;
      uv_rect = cam_model_.project3dToPixel(pt_cv);
      uv = cam_model_.unrectifyPoint(uv_rect); // compute unrectified points
      ROS_INFO("%0.2f,%0.2f,%0.2f -> %0.2f,%0.2f", pt.x(), pt.y(), pt.z(), uv.x, uv.y);

      if (save_on)
        pts.push_back(uv);

      if (view_on)
      {
        static const int RADIUS = 3;
        cv::circle(cv_ptr->image, uv, RADIUS, CV_RGB(255,0,0), -1);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(frame_id.c_str(), CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point origin = cv::Point(uv.x - text_size.width / 2,
                                 uv.y - RADIUS - baseline - 3);
        cv::putText(cv_ptr->image, frame_id.c_str(), origin, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
      }
    }

    if (save_on)
    {
      cv::Mat pts_mat(pts);
      cv::Mat pts_tmat = pts_mat.t();
      col_vals.push_back(pts_tmat.clone());
      timestamp.push_back(info_msg->header.stamp.toSec());
      writeData(frame_ids_, col_vals, timestamp);
    }

    if (view_on)
      pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  string image_topic;
  std::vector<std::string> frame_ids;

  // handling arguments
  po::options_description optionsDescription
    ( "Project tf frames to the camera\n"
      "Example:\n"
      "  project_tf_to_camera -f /left_hand_1 /right_hand1 -t /fisheye1/gscam/image_color -v\n"
      "  rosrun image_view image_view image:=/image_out\n"
      "Allowed options:");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("frame_id,f", po::value<std::vector<string> > (&frame_ids)->multitoken()->required(), "list of frame_ids to be transformed to the camera")
    ("image_topic,t", po::value<string>(&image_topic)->required(), "image topics to be processed")
    ("visualize,v", "visualize all tf in an new image (/image_out)")
    ("time_compensate,c", po::value<float>(&time_shift)->required(), "compensate timestamp for a small shift")
    ("save_to_file,s", po::value<string>(&save_file)->default_value(""), "file to store skeleton points");

  po::variables_map variablesMap;
  try
  {
    po::store(parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {std::cout<<optionsDescription<<std::endl; return 0;}
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

  view_on = (variablesMap.count("visualize") > 0);
  save_on = !save_file.empty();

  ros::init(argc, argv, "project_tf_to_camera");
  FrameDrawer drawer(frame_ids,image_topic);
  ros::spin();
}

