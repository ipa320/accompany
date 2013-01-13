#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

int view_on;

class FrameDrawer
{
  // std::string image_topic_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids, std::string image_topic)
    : it_(nh_), frame_ids_(frame_ids)
  {
    ROS_INFO("subscribed to camera %s",image_topic.c_str());
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    if (view_on)
    {
      pub_ = it_.advertise("image_out", 1);
      cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    IplImage* image = NULL;
    try {
      image = bridge_.imgMsgToCv(image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& ex) {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      tf::StampedTransform transform;
      try
      {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);
      ROS_INFO("%0.2f,%0.2f,%0.2f -> %0.2f,%0.2f", pt.x(), pt.y(), pt.z(), uv.x, uv.y);

      if (view_on)
      {
        static const int RADIUS = 3;
        cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
        CvSize text_size;
        int baseline;
        cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
        CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                                 uv.y - RADIUS - baseline - 3);
        cvPutText(image, frame_id.c_str(), origin, &font_, CV_RGB(255,0,0));
      }
    }
    if (view_on)
      pub_.publish(bridge_.cvToImgMsg(image, "bgr8"));
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
      "  project_tf_to_camera -s /left_hand_1 /right_hand1 -t /fisheye1/gscam/image_color -v\n"
      "  rosrun image_view image_view image:=/image_out\n"
      "Allowed options:");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("frame_id,s", po::value<std::vector<string> > (&frame_ids)->multitoken()->required(), "list of frame_ids to be transformed to the camera")
    ("image_topic,t", po::value<string>(&image_topic)->required(), "image topics to be processed")
    ("visualize,v", "visualize all tf in an new image (/image_out)");

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
  view_on = variablesMap.count("visualize");

  ros::init(argc, argv, "project_tf_to_camera");
  FrameDrawer drawer(frame_ids,image_topic);
  ros::spin();
}

