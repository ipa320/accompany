/***************************************************************************
 *   cameraModel.cpp     - CameraModel CLASS FUNCTIONS
 *
 *   Ninghang Hu - University of Amsterdam
 *
 *   ACCOMPANY PROJECT 2012
 *
 ***************************************************************************/

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <libxml/xmlwriter.h>
#include <libxml/xmlreader.h>

#include <cameraModel.h>

using namespace std;
using namespace Hu;

CameraModel::CameraModel()
{
  isInit = false;
}

CameraModel::~CameraModel()
{
}

void CameraModel::init(string IntrinsicFile, string ExtrinsicFile, double SCALE) //TODO
{

  /* Loading intrisic parameters */
  cv::FileStorage fs(IntrinsicFile, cv::FileStorage::READ);

  fs["camera_matrix"] >> camera_matrix;
  fs["distortion_coefficients"] >> distortion_coefficients;

  camera_matrix = camera_matrix * SCALE;
  //    distortion_coefficients = distortion_coefficients * SCALE;

  cv::FileStorage fs_extrin(ExtrinsicFile, cv::FileStorage::READ);
  fs_extrin["rvec"] >> rvec;
  fs_extrin["tvec"] >> tvec;

  cout << "rvec = " << rvec / 3.14 * 180 << " degree" << endl;
  cout << "tvec = " << tvec << endl;

  // cache
  cv::Rodrigues(rvec,worl2ImageRotMatrix);
  cout<<"worl2ImageRotMatrix:"<<worl2ImageRotMatrix<<endl;
  image2WorldRotMatrix=worl2ImageRotMatrix.inv();
  cout<<"image2WorldRotMatrix:"<<image2WorldRotMatrix<<endl;
  image2WorldTransMatrix=image2WorldRotMatrix*(tvec*-1);
  cout<<"image2WorldTransMatrix:"<<image2WorldTransMatrix<<endl;

  isInit = true;
}

//! Coordinate manipulation
//! from image coordinate to world coordinate (single points)
bool CameraModel::imageToWorld(double Xi, double Yi, double Zw, double& Xw,
    double &Yw)
{
  bool done = false;
  if (isInit)
  {
    cv::Mat image_coordinates = (cv::Mat_<double>(1, 2) << Xi, Yi);

    /* Distorted image coordinates -> Undistorted coordinates (homogenious) */
    cv::Mat undistorted_points;
    undistortPoints(image_coordinates.reshape(2), undistorted_points,
        camera_matrix, distortion_coefficients);

    /* Undistorted coordinates -> World coordinates */
    undistorted_points = undistorted_points.reshape(1);

    cv::transpose(undistorted_points, undistorted_points); // (2xN mtx)
    cv::Mat vec_ones = cv::Mat::ones(cv::Size(undistorted_points.cols, 1),
        undistorted_points.type());
    undistorted_points.push_back(vec_ones); // make it homogenius-add the third component to 1 (3xN mtx)
    
    cv::Mat homo_point=image2WorldRotMatrix * (undistorted_points - tvec);

    double Zc = image2WorldTransMatrix.at<double>(2);
    double Zh = homo_point.at<double>(2);
    double s = (Zc - Zw) / (Zc - Zh);

    cv::Mat Pts_wc = (homo_point - image2WorldTransMatrix) * s + image2WorldTransMatrix;
    //        cout << Pts_wc.rowRange(0,2) << endl;
    Xw = Pts_wc.at<double>(0);
    Yw = Pts_wc.at<double>(1);

    done = true;
  }
  return done;

}
//! from world coordinate to image coordinate
//bool CameraModel::worldToImage(double Xw, double Yw, double Zw, double& Xi, double& Yi)
bool CameraModel::worldToImage(double Xw, double Yw, double Zw, double& Xi,
    double& Yi)
{
  bool done = false;
  if (isInit)
  {
    cv::Mat world_coordinates = (cv::Mat_<double>(1, 3) << Xw, Yw, Zw);
    cv::Mat image_coordinates;

    /* World coordinates -> Distorted image coordinates */
    projectPoints(world_coordinates, rvec, tvec, camera_matrix,
        distortion_coefficients, image_coordinates);

    Xi = image_coordinates.at<double>(0, 0);
    Yi = image_coordinates.at<double>(0, 1);

    done = true;
  }
  return done;
}

//! from world coordinate to image coordinate (Matrix)
bool CameraModel::worldToImageMat(cv::Mat world_coordinates,
    cv::Mat& image_coordinates)
{
  bool done = false;
  if (isInit)
  {
    /* World coordinates -> Distorted image coordinates */
    cv::projectPoints(world_coordinates, rvec, tvec, camera_matrix,
        distortion_coefficients, image_coordinates);
    image_coordinates = image_coordinates.reshape(1);
    done = true;
  }
  return done;
}

const cv::Mat& CameraModel::getWorl2ImageRotMatrix() const
{
  return worl2ImageRotMatrix;
}

const cv::Mat& CameraModel::getImage2WorldRotMatrix() const
{
  return image2WorldRotMatrix;
}

const cv::Mat& CameraModel::getImage2WorldTransMatrix() const
{
  return image2WorldTransMatrix;
}
