#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

cv::Mat loadTxtData(string file)
{

    // pre-load data to get matrix dimension
    ifstream inFile;
    inFile.open(file.c_str(),ifstream::in);

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
            while (getline (linestream, item, ','))
            {
                num_of_column++;
            }
        }
	}
    inFile.close();

    // load data to matrix
    inFile.open(file.c_str(),ifstream::in);
    cv::Mat outMat(number_of_lines,num_of_column,CV_64F);
	int  linenum = 0;
    while (getline (inFile, line))
    {
        istringstream linestream(line);
        string item;
        int itemnum = 0;
        while (getline (linestream, item, ','))
        {
            double item_data = atoi(item.c_str());
            outMat.at<double>(linenum,itemnum) = item_data;
            itemnum++;
        }
        linenum++;
    }
    inFile.close();
    return outMat;
}

int main( int argc, char** argv )
{
	string intrinsicFile, extrinsicFile, points3dFile, points2dFile;

    // handling arguments
    po::options_description optionsDescription("Calibrate extrinsic parameters of camera, give rotation matrix and translation matrix to world frame\nAllowed options\n");
    optionsDescription.add_options()
        ("intrinsicFile,i", po::value<string>(&intrinsicFile)->required(),"the input filename for intrinsic parameter\n")
        ("extrinsicFile,o", po::value<string>(&extrinsicFile)->required(),"the output filename for extrinsic parameter\n")
        ("point2d,p", po::value<string>(&points2dFile)->required(),"the annotated 2D points on the image space\n")
        ("point3d,q", po::value<string>(&points3dFile)->required(),"the corresponding 3D coordinates in the world frame\n")
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

	cv::Mat camera_matrix, distortion_coefficients, rvec, tvec;
	
	/* Loading extrinsic parameters*/
    cv::Mat world_coordinates = loadTxtData(points3dFile); // Nx3
    cv::Mat image_coordinates = loadTxtData(points2dFile); // Nx2

    // Load Intrinsic Parameters
    cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coefficients;

	world_coordinates.convertTo(world_coordinates,CV_32F);
	image_coordinates.convertTo(image_coordinates,CV_32F);
	cv::solvePnP(world_coordinates, image_coordinates, camera_matrix,distortion_coefficients, rvec, tvec);

	cv::FileStorage fs_extrinsic(extrinsicFile, cv::FileStorage::WRITE);
	fs_extrinsic << "rvec" << rvec;
	fs_extrinsic << "tvec" << tvec;
	
	cout << endl;
	cout << "extrinsic parameters saved to " << extrinsicFile << endl;
}

