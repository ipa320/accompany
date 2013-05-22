#include <cameraModel.h>
using namespace Hu;

#include <iostream>
using namespace std;

int main(int argc,char **argv)
{
  CameraModel cameraModel;

  cameraModel.init(argv[1], argv[2], 1);

  
  double Xi=1.123;
  double Yi=2.124;
  double Zw=1.086897;
  double Xw,Yw;
  cameraModel.imageToWorld(Xi,Yi,Zw,
                           Xw,Yw);

  cout<<"Xw:"<<Xw<<endl;
  cout<<"Yw:"<<Yw<<endl;

  /*
  cv::Mat image_coordinates=(cv::Mat_<double>(1, 3) << Xi,Yi,Zw);
  cv::Mat world_coordinates;
  cameraModel.imageToWorldMat(image_coordinates,
                              world_coordinates);
  cout<<world_coordinates<<endl;
  */

  return 1;
}
