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


void CameraModel::init(string IntrinsicFile,string ExtrinsicFile,double SCALE) //TODO
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
    
    cout << "rvec = " << rvec / 3.14 * 180 << " degree"<< endl;
    cout << "tvec = " << tvec << endl;
    
    isInit = true;
}

bool CameraModel::fromXml(string IntrinsicFile,string ExtrinsicFile,double SCALE) //TODO
{
    init(IntrinsicFile,ExtrinsicFile,SCALE);
	return isInit;
}

//! Coordinate manipulation
//! from image coordinate to world coordinate (single points)
bool CameraModel::imageToWorld(double Xi, double Yi, double Zw, double& Xw, double &Yw)
{
	bool done = false;
	if (isInit)
	{
		cv::Mat image_coordinates = (cv::Mat_<double>(1,2) << Xi,Yi);

		 /* Distorted image coordinates -> Undistorted coordinates (homogenious) */
		cv::Mat undistorted_points;
		undistortPoints(image_coordinates.reshape(2),undistorted_points,camera_matrix,distortion_coefficients);


		/* Undistorted coordinates -> World coordinates */
		cv::Mat Rotation_mtx;
		cv::Rodrigues(rvec,Rotation_mtx); // Rotation vector 3x1 to Rotation matrix 3x3 //TODO
		undistorted_points = undistorted_points.reshape(1);

		cv::transpose(undistorted_points,undistorted_points); // (2xN mtx)
		cv::Mat vec_ones = cv::Mat::ones(cv::Size(undistorted_points.cols,1),undistorted_points.type());
		undistorted_points.push_back(vec_ones); // make it homogenius-add the third component to 1 (3xN mtx)
		
        cv::Mat camera_ic = cv::Mat::zeros(tvec.size(),tvec.type());
        cv::Mat camera_wc = Rotation_mtx.inv() * (camera_ic - tvec);
        cv::Mat homo_point = Rotation_mtx.inv() * (undistorted_points - tvec);

        double Zc = camera_wc.at<double>(2);
        double Zh = homo_point.at<double>(2);
        double s = (Zc - Zw) / (Zc - Zh);

        cv::Mat Pts_wc = (homo_point - camera_wc) * s + camera_wc;
//        cout << Pts_wc.rowRange(0,2) << endl;
        Xw = Pts_wc.at<double>(0);
        Yw = Pts_wc.at<double>(1);

		done = true;
	}
	return done;

}
//! from world coordinate to image coordinate
//bool CameraModel::worldToImage(double Xw, double Yw, double Zw, double& Xi, double& Yi)
bool CameraModel::worldToImage(double Xw, double Yw, double Zw, double& Xi, double& Yi)
{
	bool done = false;
	if (isInit)
	{
		cv::Mat world_coordinates = (cv::Mat_<double>(1,3) << Xw,Yw,Zw);
		cv::Mat image_coordinates;

		/* World coordinates -> Distorted image coordinates */
		projectPoints(world_coordinates, rvec, tvec, camera_matrix,distortion_coefficients,image_coordinates);

		Xi = image_coordinates.at<double>(0,0);
		Yi = image_coordinates.at<double>(0,1);

		done = true;
	}
	return done;
}

//! from image coordinate to world coordinate (Matrix)
bool CameraModel::imageToWorldMat(cv::Mat image_coordinates, cv::Mat& world_coordinates)
{
	bool done = false;

	if (isInit)
	{
		 /* Distorted image coordinates -> Undistorted coordinates (homogenious) */
		cv::Mat undistorted_points;
		cv::undistortPoints(image_coordinates.reshape(2),undistorted_points,camera_matrix,distortion_coefficients);


		/* Undistorted coordinates -> World coordinates */
		cv::Mat Rotation_mtx;
		cv::Rodrigues(rvec,Rotation_mtx); // Rotation vector 3x1 to Rotation matrix 3x3
		undistorted_points = undistorted_points.reshape(1);
		cv::transpose(undistorted_points,undistorted_points); // (2xN mtx)
		cv::Mat vec_ones = cv::Mat::ones(cv::Size(undistorted_points.cols,1),undistorted_points.type());
		undistorted_points.push_back(vec_ones); // make it homogenius-add the third component to 1 (3xN mtx)

		// concatenate translation matrix
		cv::Mat _tvec = tvec.t(); // transition vector (1x3)
		cv::Mat tvec_Mtx = _tvec.clone();
		for (int i=0;i<undistorted_points.cols-1;i++)
		{
			tvec_Mtx.push_back(_tvec);
		}
		transpose(tvec_Mtx,tvec_Mtx); // Translation Matrix 3xN

		// camera coordinates -> world coordinates
		cv::Mat Pts_wc = Rotation_mtx.inv() * (undistorted_points - tvec_Mtx); // homogenius points in world coordinations
		cv::Mat _cam = (Rotation_mtx.inv() * (tvec * (-1))).t(); // location of Camera C = R.inv * (0-tvec) (1x3 matrix)

		// concatenate camera matrix
		cv::Mat CAM_MTX = _cam.clone();
		for (int i=0;i<undistorted_points.cols-1;i++)
		{
			CAM_MTX.push_back(_cam);
		}
		transpose(CAM_MTX,CAM_MTX); // Camera location matrix Nx3

		// project points to ground plane
		cv::Mat A = CAM_MTX.rowRange(0,2);
		cv::Mat Z0 = CAM_MTX.row(2);

		cv::Mat B = Pts_wc.rowRange(0,2);
		cv::Mat Z1 = Pts_wc.row(2);

		A.row(0) = A.row(0).mul(Z1);
		A.row(1) = A.row(1).mul(Z1);
		B.row(0) = B.row(0).mul(Z0);
		B.row(1) = B.row(1).mul(Z0);

		cv::Mat C = A - B;
		C.row(0) = C.row(0) / (Z1-Z0);
		C.row(1) = C.row(1) / (Z1-Z0);
		transpose(C,C);

		world_coordinates = C.clone();
		done = true;
	}
	return done;
}

//! from world coordinate to image coordinate (Matrix)
bool CameraModel::worldToImageMat(cv::Mat world_coordinates, cv::Mat& image_coordinates)
{
	bool done = false;
	if (isInit)
	{
		/* World coordinates -> Distorted image coordinates */
		cv::projectPoints(world_coordinates, rvec, tvec, camera_matrix,distortion_coefficients,image_coordinates);
		image_coordinates = image_coordinates.reshape(1);
		done = true;
	}
	return done;
}


