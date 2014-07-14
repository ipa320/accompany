#include <stdlib.h>
#include <unistd.h>

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <opencv2/opencv.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//forward declarations
//static gboolean processData(GstPad *pad, GstBuffer *buffer, gpointer u_data);
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

//globals
bool gstreamerPad, rosPad;
int width, height;
sensor_msgs::CameraInfo camera_info;

void xml2yaml(std::string calib_xml, std::string& calib_yaml)
{
  int image_width, image_height;
  std::string camera_name, distortion_model;
  cv::Mat camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix;

  boost::filesystem::path p(calib_xml);
  boost::filesystem::path dir = p.parent_path();
  // calib_yaml = dir.string() + "/camera_intrinsic.yaml";
  calib_yaml = "camera_intrinsic.yaml";

  cv::FileStorage fs1(calib_xml, cv::FileStorage::READ);
  cv::FileStorage fs2(calib_yaml,cv::FileStorage::WRITE);
  if (!fs1.isOpened())
  {
    ROS_ERROR("cannot open %s", calib_xml.c_str());
  }

  ROS_INFO("convert %s to %s", calib_xml.c_str(), calib_yaml.c_str());
  fs1["image_width"] >> image_width;
  fs2 << "image_width" << image_width;
  fs1["image_height"] >> image_height;
  fs2 << "image_height" << image_height;
  fs2 << "camera_name" << "camera";
  fs1["camera_matrix"] >> camera_matrix;
  fs2 << "camera_matrix" << camera_matrix;

  fs2 << "distortion_model" << "plumb_bob";
  fs1["distortion_coefficients"] >> distortion_coefficients;
  fs2 << "distortion_coefficients" << distortion_coefficients;

  cv::hconcat(camera_matrix,cv::Mat::zeros(3,1,CV_64F),projection_matrix);

  fs2 << "rectification_matrix" << cv::Mat::eye(3,3,CV_32F);
  fs2 << "projection_matrix" << projection_matrix;

  fs1.release();
  fs2.release();

}

int main(int argc, char** argv)
{
  bool sync;
  std::string frame_id, calib_xml, calib_yaml;

  // arguments
  po::options_description optionsDescription(
      " GSCAM: camera driver\n"
      " example: rosrun gscam gscam -s 0 -f gscam_optical_frame -i camera_intrinsic.xml\n"
      " start camera without synchronization and load intrinsic parameters\n"
      " use image_proc to produce rectified image (undistortion)\n"
      "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("sync,s", po::value<bool>(&sync)->default_value(true),"parent frame\n")
    ("frame_id,f", po::value<std::string>(&frame_id)->default_value("gscam_optical_frame"),"assign camera frame with frame_id\n")
    ("intrinsic_xml,i", po::value<std::string>(&calib_xml)->default_value(""),"load intrinsic parameters, use image_proc for image undistortion\n");

  po::variables_map variablesMap;

  try
  {
    po::store(po::parse_command_line(argc, argv, optionsDescription),variablesMap);
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

  char *config = getenv("GSCAM_CONFIG");
  if (config == NULL) {
    std::cout << "Problem getting GSCAM_CONFIG variable." << std::endl;
    exit(-1);
  }

  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh;
  std::string resolved_gscam = nh.resolveName("gscam");// allows users to rename topics
  std::cout<<"publishing to topic name: '"<<resolved_gscam<<"'"<<std::endl;

  gst_init(0,0);
  std::cout << "Gstreamer Version: " << gst_version_string() << std::endl;

  GError *error = 0; //assignment to zero is a gst requirement
  GstElement *pipeline = gst_parse_launch(config,&error);
  if (pipeline == NULL) {
    std::cout << error->message << std::endl;
    exit(-1);
  }
  GstElement * sink = gst_element_factory_make("appsink",NULL);
  GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb", NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink), caps);
  gst_caps_unref(caps);

  if (sync)
    gst_base_sink_set_sync(GST_BASE_SINK(sink),TRUE);
  else
    gst_base_sink_set_sync(GST_BASE_SINK(sink),FALSE);

  if(GST_IS_PIPELINE(pipeline)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline), GST_PAD_SRC);
    g_assert(outpad);
    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);


    if(!gst_bin_add(GST_BIN(pipeline), sink)) {
      fprintf(stderr, "gst_bin_add() failed\n"); // TODO: do some unref
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      return -1;
    }

    if(!gst_element_link(outelement, sink)) {
      fprintf(stderr, "GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      return -1;
    }

    gst_object_unref(outelement);
  } else {
    GstElement* launchpipe = pipeline;
    pipeline = gst_pipeline_new(NULL);
    g_assert(pipeline);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline), launchpipe, sink, NULL);

    if(!gst_element_link(launchpipe, sink)) {
      fprintf(stderr, "GStreamer: cannot link launchpipe -> sink\n");
      gst_object_unref(pipeline);
      return -1;
    }
  }

  gst_element_set_state(pipeline, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    std::cout << "Failed to PAUSE." << std::endl;
    exit(-1);
  } else {
    std::cout << "stream is PAUSED." << std::endl;
  }

  // We could probably do something with the camera name, check
  // errors or something, but at the moment, we don't care.
  std::string camera_name;

  camera_info.header.frame_id = frame_id;

//  if (variablesMap.count("intrinsic_xml"))
  if (!calib_xml.empty())
  {
    ROS_INFO("loading calibration file %s", calib_xml.c_str());

    xml2yaml(calib_xml,calib_yaml);

    if (camera_calibration_parsers::readCalibrationYml(calib_yaml, camera_name, camera_info))
    {
      ROS_INFO("Successfully read camera calibration. Return camera calibrator if it is incorrect.");
      if (variablesMap.count("frame_id") == 0)
        ROS_WARN("No frame_id specified, use default: %s\n Use option -f to specify which frame to associate", frame_id.c_str());
      else
        ROS_INFO("Use the specified frame_id: %s", frame_id.c_str());
    }
    else
    {
      ROS_WARN("No camera_parameters.txt file found.  Use default file if no other is available.");
      ROS_WARN("Use -i option to load the intrinsic parameters");
    }
  }


  int preroll;
  nh.param("brown/gscam/preroll", preroll, 0);
  if (preroll) {
    //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    //I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      std::cout << "Failed to PLAY." << std::endl;
      exit(-1);
    } else {
      std::cout << "stream is PLAYING." << std::endl;
    }

    gst_element_set_state(pipeline, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      std::cout << "Failed to PAUSE." << std::endl;
      exit(-1);
    } else {
      std::cout << "stream is PAUSED." << std::endl;
    }
  }

  image_transport::ImageTransport it(nh);

  image_transport::CameraPublisher pub = it.advertiseCamera(resolved_gscam+"/image_raw", 1);
  ros::ServiceServer set_camera_info = nh.advertiseService(resolved_gscam+"/set_camera_info", setCameraInfo);

  std::cout << "Processing..." << std::endl;

  //processVideo
  rosPad = false;
  gstreamerPad = true;
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  while(nh.ok()) {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink));
    if (!buf) break;

    GstPad* pad = gst_element_get_static_pad(sink, "sink");
    const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps,0);
    gst_structure_get_int(structure,"width",&width);
    gst_structure_get_int(structure,"height",&height);

    sensor_msgs::Image msg;

    // load camera info to image msg
    camera_info.header.stamp = ros::Time::now();
    msg.header.stamp = camera_info.header.stamp;
    msg.header.frame_id = camera_info.header.frame_id;

    msg.width = width;
    msg.height = height;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width*3;
    msg.data.resize(width*height*3);
    std::copy(buf->data, buf->data+(width*height*3), msg.data.begin());

    pub.publish(msg, camera_info);

    gst_buffer_unref(buf);

    ros::spinOnce();

  }

  //close out
  std::cout << "\nquitting..." << std::endl;
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);

  return 0;
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

  ROS_INFO("New camera info received");
  camera_info = req.camera_info;

  if (camera_calibration_parsers::writeCalibrationIni("../camera_parameters.txt", "gscam", camera_info)) {
    ROS_INFO("Camera information written to camera_parameters.txt");
    return true;
  }
  else {
    ROS_ERROR("Could not write camera_parameters.txt");
    return false;
  }
}
