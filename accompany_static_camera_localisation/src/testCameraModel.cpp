#include "cameraModel.h"
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;

cv::Mat loadTxtData(string file)
{

  // pre-load data to get matrix dimension
  ifstream inFile;
  inFile.open(file.c_str(), ifstream::in);

  if (!inFile)
  {
    cout << "Error: \"" << file << "\" not found " << endl;
    exit(0);
  }

  string line;
  int number_of_lines = 0, num_of_column = 0;
  while (std::getline(inFile, line))
  {
    ++number_of_lines;
    if (number_of_lines <= 1)
    {
      istringstream linestream(line);
      string item;
      while (getline(linestream, item, ','))
      {
        num_of_column++;
      }
    }
  }
  inFile.close();

  // load data to matrix
  inFile.open(file.c_str(), ifstream::in);
  cv::Mat outMat(number_of_lines, num_of_column, CV_64F);
  int linenum = 0;
  while (getline(inFile, line))
  {
    istringstream linestream(line);
    string item;
    int itemnum = 0;
    while (getline(linestream, item, ','))
    {
      double item_data = atoi(item.c_str());
      outMat.at<double>(linenum, itemnum) = item_data;
      itemnum++;
    }
    linenum++;
  }
  inFile.close();
  return outMat;
}

int main()
{

	
//    
//	cout << wc1000 < endl;

//    cout << "Projection error: World->Image (pixels)" << endl;
//	cam.worldToImageMat(wc,ic2);

//	cv::Mat distic = ic.colRange(0,2) - ic2;
//	cv::pow(distic,2,distic);
//	cv::Mat distic_sum = distic.col(0) + distic.col(1);
//	cv::sqrt(distic_sum,distic_sum);
//	for (int i = 0;i<distic_sum.rows;i++)
//	{
//	    cout << distic_sum.row(i) << endl;
//    }
    



//    cout << "Projection error: Image->World (mm)" << endl;
//	cam.imageToWorldMat(ic,wc2);
//	cout << ic << endl << wc2 << endl;
//	
//	
//	cv::Mat distwc = wc.colRange(0,2) - wc2;
//	cv::pow(distwc,2,distwc);
//	cv::Mat distwc_sum = distwc.col(0) + distwc.col(1);
//	cv::sqrt(distwc_sum,distwc_sum);
//	for (int i = 0;i<distwc_sum.rows;i++)
//	{
//	    cout << distwc_sum.row(i) << endl;
//    }
//    
//    cout << "ic" << endl;
//    cout << ic << endl;
//    cout << "wc" << endl;
//    cout << wc << endl;


}

