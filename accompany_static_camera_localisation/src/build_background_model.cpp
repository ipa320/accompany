#include <iostream>
#include "Background.hh"
#include "Helpers.hh"
#include <cmn/random.hh>
#include <tools/string.hh>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <data/XmlFile.hh>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

#define TO_IMG_FMT CV_Luv2BGR
#define TO_INT_FMT CV_BGR2Luv

FLOAT logGaus(FLOAT sqdiff, FLOAT sigmasq)
{
	return -0.5 * (log(2 * M_PI * sigmasq) + sqdiff/sigmasq);
}

inline FLOAT gaussProb(FLOAT sqdiff, FLOAT sigmasq)
{
	return exp(logGaus(sqdiff, sigmasq));
}

int main(int argc, char **argv) {

	string imagelist_training, output_background_model;
	int num_components, num_frames;

	// handling arguments
	po::options_description optionsDescription("Build the eigen-background model for background subtraction\nAllowed options\n");
	optionsDescription.add_options()
		("imagelist_training,i", po::value<string>(&imagelist_training)->required(),"the input image list for training\n")
		("output_background_model,o", po::value<string>(&output_background_model)->required(),"the output background model\n")
		("num_components,c", po::value<int>(&num_components)->default_value(3),"the number of components\n")
		("num_frames,f", po::value<int>(&num_frames)->default_value(0),"the number of components\n")
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

	vector< vector<string> > trainset;

	const char* s_trainset = imagelist_training.c_str();
	const char* s_outfile = output_background_model.c_str();
	//     int s_numcomp = num_components;
	//     int s_numframes = num_frames; // may be invalid


	cout << "Listing images" << endl;
	listImages(s_trainset,trainset);
	cout << "Done." << endl;
	unsigned
	N=(unsigned)(0.8*trainset.size());
	vector<Background>
	bgModel(0, Background(0,0));

	int C;

	if (num_frames == 0) {
		C = num_components;
		bgModel = vector<Background>(trainset[0].size(),Background(N,C));

		for (unsigned j=0; j!=trainset[0].size(); ++j) {

			width = 0;       // new file size is possible;
			for (unsigned i=0; i!=trainset.size(); ++i) {
				string filename = trainset[i][j];
				cout << "Loading " << filename << endl;
				IplImage *src = cvLoadImage(filename.c_str());
				IplImage *img = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
				//cvSmooth(src,src,CV_GAUSSIAN,2,2);
				cvCvtColor(src,img,TO_INT_FMT);

				// cvSplit(img,ch0,ch1,ch2,NULL);
				// cvFilter2D(ch0,ch0,gausKernel);
				// cvFilter2D(ch1,ch1,gausKernel);
				// cvFilter2D(ch2,ch2,gausKernel);
				// cvMerge(ch0,ch1,ch2,NULL,img);
				cvReleaseImage(&src);
				// cvReleaseImage(&ch0);
				// cvReleaseImage(&ch1);
				// cvReleaseImage(&ch2);

				vnl_vector<FLOAT>
				imgV;
				img2vec(img, imgV);
				// cout << imgV.extract(5) << endl;
				bgModel[j].processImage(imgV);
				// cvReleaseImage(&src);
				cvReleaseImage(&img);
			}
			cout << "Updating " << j << endl;
			bgModel[j].update();
			bgModel[j].dropImg();
			cout << "Done." << endl;
		}
	}
	else if (num_frames > 0) {
		width = 0;            // new image size is possible
		N = num_frames;
		bgModel = vector<Background>(trainset[0].size(), Background(N,num_components));
		for (unsigned j=0; j!=trainset[0].size(); ++j) {
			for (unsigned i=0; i!=trainset.size(); ++i) {
				string filename = trainset[rand_range(0,trainset.size()-1)][j];
				cout << "Loading " << filename << endl;
				IplImage *src = loadImage(filename.c_str());
				IplImage *img = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
				cvCvtColor(src,img,TO_INT_FMT);
				cvReleaseImage(&src);

				vnl_vector<FLOAT>
				imgV;
				img2vec(img, imgV);
				cout << imgV.extract(5) << endl;
				bgModel[j].processImage(imgV);
				// cvReleaseImage(&src);
				cvReleaseImage(&img);
			}
			cout << "Updating model " << j << " of " << bgModel.size() << endl;
			bgModel[j].update();
			bgModel[j].dropImg();
			cout << "Done." << endl;
		}
	}

	XmlWriter
	wr(s_outfile);
	wr.pack("Background",bgModel);

	return 0;
}
