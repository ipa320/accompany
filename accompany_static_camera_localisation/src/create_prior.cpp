#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <err.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "Helpers.hh"
#include "CamCalib.hh"
#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace std;

CvScalar CLR = CV_RGB(0,255,0);
vector<IplImage *> img;
vector<WorldPoint> priorHull;

const char *win[] = { "1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20"};

void plotHull(IplImage *img, unsigned idx)
{
	vector<CvPoint> proj;
	for (unsigned i=0; i!=priorHull.size(); ++i)
		proj.push_back(cam[idx].project(priorHull[i]));
	proj.push_back(proj.front());

	cvCircle(img, proj[0],2, CLR, 1);
	for (unsigned i=1; i<proj.size(); ++i) {
		cvCircle(img,proj[i],5,CLR,3);
		cvLine(img, proj[i-1],proj[i],CLR,2);
	}
}

void plotHull(IplImage *img, unsigned idx, const WorldPoint &pt)
{
	vector<CvPoint> proj;
	for (unsigned i=0; i!=priorHull.size(); ++i)
		proj.push_back(cam[idx].project(priorHull[i]));
	proj.push_back(cam[idx].project(pt));
	proj.push_back(proj.front());

	cvCircle(img, proj[0],5, CV_RGB(0,255,0), 3);
	for (unsigned i=1; i<proj.size(); ++i) {
		cvCircle(img,proj[i],5,CLR,3);
		cvLine(img,proj[i-1],proj[i],CLR,2);
	}

}

void mouseHandler(int idx, int event, int x, int y, int flags, void *)
{
	WorldPoint pt = cam[idx].getGroundPos(cvPoint(x,y));

	static bool down=false;

	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		cout << "Left button down at " << pt.x << "," << pt.y << endl;
		cout << "press 'q' to finish and save to file" << endl;
		down = true;
		for (unsigned i=0; i!=img.size(); ++i) {
			IplImage
			*tmp = cvCloneImage(img[i]);
			plotHull(tmp,i,pt);
			cvShowImage(win[i], tmp);
			cvReleaseImage(&tmp);
		}
		break;
	case CV_EVENT_MOUSEMOVE:
		if (down)
			for (unsigned i=0; i!=img.size(); ++i) {
				IplImage
				*tmp = cvCloneImage(img[i]);
				plotHull(tmp,i,pt);

				cvShowImage(win[i],tmp);
				cvReleaseImage(&tmp);
			}
		break;
	case CV_EVENT_LBUTTONUP:
		down = false;
		priorHull.push_back(pt);
		cout << "Up at (" << pt.x << "," << pt.y << ")" << endl;
		for (unsigned i=0; i!=img.size(); ++i) {
			IplImage
			*tmp = cvCloneImage(img[i]);
			plotHull(tmp,i);
			cvShowImage(win[i],tmp);
			cvReleaseImage(&tmp);
		}

		break;
	}
}

#define DEF(IDX) void mh##IDX(int e, int x, int y, int f, void *p) { return mouseHandler(IDX,e,x,y,f,p); }
DEF(0) DEF(1) DEF(2) DEF(3) DEF(4) DEF(5) DEF(6) DEF(7) DEF(8) DEF(9) DEF(10)
		DEF(11) DEF(12) DEF(13) DEF(14) DEF(15) DEF(16) DEF(17)DEF(18) DEF(19) DEF(20)
		typedef void (*mh_t)(int,int,int,int,void*);
mh_t mh[] = { mh0,mh1,mh2,mh3,mh4,mh5,mh6,mh7,mh8,mh9,mh10,mh11,mh12,mh13,mh14,mh15,mh16,mh17,mh18,mh19,mh20 };



int main(int argc, char **argv) {

	string imagelist_file, params_file, outputPrior_file;

	// handling arguments
	po::options_description optionsDescription("Select prior locations where people can walk\nAllowed options\n");
	optionsDescription.add_options()
        		("imagelist,i", po::value<string>(&imagelist_file)->required(),"the input image list showing the ground plane\n")
        		("params,p", po::value<string>(&params_file)->required(),"the input xml file containing all parameters\n")
        		("outputPrior,o", po::value<string>(&outputPrior_file)->required(),"the output filename of the prior\n")
        		;

	po::variables_map variablesMap;

	try
	{
		po::store(po::parse_command_line(argc, argv, optionsDescription), variablesMap);
		po::notify(variablesMap);
	}
	catch( const std::exception& e)
	{
		std::cout << "--------------------" << std::endl;
		std::cerr << "- "<<e.what() << std::endl;
		std::cout << "--------------------" << std::endl;
		std::cout <<  optionsDescription << std::endl;
		return 1;
	}

	const char* arg_imagelist_file = imagelist_file.c_str();
	const char* arg_params_file = params_file.c_str();

	vector< vector<string> >
	imgs;
	listImages(arg_imagelist_file,imgs);
	cam = vector<CamCalib>(imgs[0].size());
	img = vector<IplImage *>(imgs[0].size());

	unsigned
	index = 0;

	for (unsigned i=0; i!=imgs[index].size(); ++i) {
		img[i] = loadImage(imgs[index][i].c_str());

		cvNamedWindow(win[i]);
		cvSetMouseCallback(win[i], mh[i], NULL);
		cvShowImage(win[i], img[i]);
	}
	width = img[0]->width;
	height = img[0]->height;
	depth = img[0]->depth;
	channels = img[0]->nChannels;
	halfresX = width/2;
	halfresY = height/2;

	loadCalibrations(arg_params_file);

	//     if (argc == 4) {
		//          loadWorldPriorHull(argv[3],priorHull);
		//          for (unsigned i=0; i!=img.size(); ++i)
	//               plotHull(img[i],i);

	//          vector<WorldPoint>
	//               scan;
	//          genScanLocations(priorHull, 100, scan);
	//          for (unsigned j=0; j!=img.size(); ++j) {
	//               for (unsigned i=0; i!=scan.size(); ++i) {
	//                    cvCircle(img[j], cam[j].project(scan[i]), 1, CLR, 1);
	//                    vector<CvPoint> tplt;
	//                    cam[j].genTemplate(scan[i],tplt);
	//                    unsigned
	//                         minX = (unsigned)-1, maxX = 0,
	//                         minY = (unsigned)-1, maxY = 0;
	//                    for (unsigned u=0; u!=tplt.size(); ++u) {
	//                         if (tplt[u].x < minX)
	//                              minX = tplt[u].x;
	//                         if (tplt[u].x > maxX)
	//                              maxX = tplt[u].x;
	//                         if (tplt[u].y < minY)
	//                              minY = tplt[u].y;
	//                         if (tplt[u].y > maxY)
	//                              maxY = tplt[u].y;
	//                    }
	//                    if (minX > width-1)
	//                         minX = 0;
	//                    if (maxX > width-1)
	//                         maxX = width-1;
	//                    if (minY > height-1)
	//                         minY = height-1;
	//                    if (maxY>height-1)
	//                         maxY = height-1;
	//                    cout << "RECTANGLE " << j << " " << i << " " << minX << " " << minY << " "
	//                         << maxX << " " << maxY << endl;
	//               }
	//
	//               cvShowImage(win[j],img[j]);
	//          }
	//     }


	int key = 0;
	while ((char)key != 'q') {
		// cvShowImage("image", src);
		key = cvWaitKey(0);
	}

	cout << "1" << endl;
	for (unsigned i=0; i!=priorHull.size(); ++i)
		cout << priorHull[i].x << " " << priorHull[i].y << " " << priorHull[i].z << endl;

	ofstream outfile;
	outfile.open(outputPrior_file.c_str());
	outfile << ("1\n");
	for (unsigned i=0; i!=priorHull.size(); ++i)
	{
		outfile << priorHull[i].x << " " << priorHull[i].y
				<< " " << priorHull[i].z << endl;
	}
	outfile.close();
	cout << endl;
	cout << "prior saved to " << outputPrior_file << endl;
	for (unsigned i=0; i!=img.size(); ++i)
		cvReleaseImage(&img[i]);
	return 0;
}
