#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <boost/program_options.hpp>

using namespace boost::program_options;
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  string src_im, calib_file, image_path;

  // handling arguments
  options_description optionsDescription
    ("Show undistorted image given the intrinsic parameter of the camera\n"
     "Allowed options");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("intrinsic_file,i", value<string>(&calib_file)->default_value("camera_intrinsic.xml"),"filename of the intrinsic parameter")
    ("src_im,s", value<string>(&src_im)->required(),"filename of the image to be undistorted")
    ("view_full_image,f", "filename of the image to be undistorted")
    ("save_image,w", value<string>(&image_path)->default_value("/home/ninghang/tmp/test.jpg"),"path to save undistorted images");

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

  cout << "loading intrinsic file: " << calib_file << endl;
  FileStorage fs(calib_file, FileStorage::READ);
  if( !fs.isOpened() )
  {
    cout << calib_file << ": file not found, check the file again or use -i to indicate a new calibration file";
    return -1;
  }
  Mat cameraMatrix, distCoeffs;
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;


  cout << "loading image: " << src_im << endl;
  Mat view, rview, map1, map2;
  view = imread(src_im);
  if(! view.data )                              // Check for invalid input
  {
    cout <<  "Could not open or find the image: " << src_im << endl ;
    return -1;
  }

  // undistortion
  if (variablesMap.count("view_full_image"))
  {
    Size imageSize = view.size();

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);
    remap(view, rview, map1, map2, INTER_LINEAR);
  }
  else
  {
    undistort( view, rview, cameraMatrix, distCoeffs);
  }

  // view
  namedWindow( "original", CV_WINDOW_NORMAL );
  imshow("original",view);
  namedWindow( "undistorted", CV_WINDOW_NORMAL );
  imshow("undistorted",rview);
  if (variablesMap.count("save_image"))
  {
    imwrite(image_path,rview);
  }

  waitKey(0);
  return 0;
}
