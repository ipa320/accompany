/*this creates a txt list of files from the command line args
 */

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include "fstream"
#include <iostream>

using std::string;
using std::cout;
using std::endl;

using namespace cv;
using namespace std;

void help(char** av)
{
	cout << "\nThis creates a txt list of files from the command line args\n"
			"usage:\n./" << av[0] << " imagelist.txt *.jpg"<< endl;
}

int main(int ac, char** av)
{
	if (ac < 3)
	{
		help(av);
		return 1;
	}

	char* outputname = av[1];

	Mat m = imread(outputname); //check if the output is an image - prevent overwrites!
	if(!m.empty()){
		std::cerr << "fail! Please specify an output file, don't want to overwrite you images!" << endl;
		help(av);
		return 1;
	}

	ofstream fs;
	fs.open(outputname);
	for(int i = 2; i < ac; i++){
		fs << av[i] << endl;
	}
	fs.close();
	return 0;
}

