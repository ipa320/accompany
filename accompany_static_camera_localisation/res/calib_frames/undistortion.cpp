#include<opencv2/opencv.hpp>
#include<stdio.h>
#include<string.h>
#include<iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  string src_im, calib_file;

  if (argc < 3)
  {
    cout << "usage: " << argv[0] << " <calib-yml>" << " <source-image>" << endl;
    exit(0);
  }

  
  calib_file = argv[1];
  
  for (unsigned int i=2;i<argc;i++)
  {
      src_im = argv[i];
      cout << src_im << endl;
      FileStorage fs(calib_file, FileStorage::READ);
      Mat cameraMatrix, distCoeffs;
      fs["camera_matrix"] >> cameraMatrix;
      fs["distortion_coefficients"] >> distCoeffs;
      
      Mat view, rview, map1, map2;
      view = imread(src_im);
      Size imageSize = view.size();

//      initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
//          getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
//          imageSize, CV_16SC2, map1, map2);
//      remap(view, rview, map1, map2, INTER_LINEAR);
      
      undistort( view, rview, cameraMatrix, distCoeffs);
      
      namedWindow( "original", CV_WINDOW_NORMAL );
      imshow("original",view);
      namedWindow( "undistorted", CV_WINDOW_NORMAL );
      imshow("undistorted",rview);
//      imwrite("test.jpg",rview);
      waitKey(0);
  }
}
