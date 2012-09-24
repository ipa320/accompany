#include "cameraModel.h"

using namespace std;

int main()
{

	string calib_file = "/home/ninghang/workspace/CameraCalib/camera.xml";
	string pnts_3D_file = "/home/ninghang/Software/PeopleDetectionGwenn/src_build/dots_on_earth.txt";
	string pnts_2D_file = "/home/ninghang/Software/PeopleDetectionGwenn/src_build/dots_on_floor.txt";

	Hu::CameraModel cam;

	cam.init();


	cv::Mat wc, ic, wc2, ic2;
	double xwc, ywc, xic, yic;
	ic = cam.loadTxtData(pnts_2D_file);
	wc = cam.loadTxtData(pnts_3D_file);

	cout << "ic" << ic << endl;
	cout << "wc" << wc << endl;

    cout << "= ImageToWorld =" << endl;
    for (int i = 0;i < ic.rows;i++)
	{
	    cam.imageToWorld(ic.at<double>(i,0),ic.at<double>(i,1),wc.at<double>(i,2),xwc,ywc);
	    cout << xwc << "," << ywc << endl;
    }    

    cout << "= WorldToImage =" << endl;
    for (int i = 0;i < wc.rows;i++)
    {
    	cam.worldToImage(wc.at<double>(i,0),wc.at<double>(i,1),wc.at<double>(i,2),xic,yic);
	    cout << xic << "," << yic << endl;
	}
	
	cv::Mat wc1000;
	wc1000 = wc.clone();
	cout << endl;
	wc1000.col(2) += 1000;
	cout << wc1000 << endl;
    for (int i = 0;i < wc.rows;i++)
    {
    	cam.worldToImage(wc1000.at<double>(i,0),wc1000.at<double>(i,1),wc1000.at<double>(i,2),xic,yic);
	    cout << xic << "," << yic << endl;
	}
	
	cout << "plotting boxes" << endl;
	cam.worldToImage(2950,950,0,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(2950,1050,0,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(3050,950,0,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(3050,1050,0,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(2950,950,1760,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(2950,1050,1760,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(3050,950,1760,xic,yic);
    cout << xic << "," << yic << endl;
	cam.worldToImage(3050,1050,1760,xic,yic);
    cout << xic << "," << yic << endl;
	
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

