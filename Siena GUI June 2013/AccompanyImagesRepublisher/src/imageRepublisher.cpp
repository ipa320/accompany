#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

//head constants
#define MAX_HEAD_POS 0.0001
#define MIN_HEAD_POS -3.1417

//image rate constants
#define RATE 4        // 3 limit number on my netWork in Siena

//image size default values
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGTH 752

//default topics
#define DEFAULT_OUTPUT_TOPIC "accompany/GUIimage"
#define DEFAULT_INPUT_TOPIC "/stereo/left/image_color"
#define DEFAULT_HEAD_TOPIC "head_controller/command"

//Image & image_transports stuffs 
IplImage* last_img;
image_transport::Publisher pub;
int count=0;
int width,heigth;  //desired output image dimensions
int half_width,half_heigth;

//images sampling ratio
int sampling_rate;

//head position 
double head_position=-3.14;

//input & output topic
std::string input_topic,output_topic;
std::string head_controller_topic;


CvMat* myCvGetRotationMatrix(CvPoint2D32f center, CvMat* matrix)
{
    double m[2][3];
    CvMat M =cvMat(2,3,CV_64FC1, m);
    double alpha,beta;
    double angle=CV_PI;
    alpha= cos(angle);
    beta= sin(angle);
    m[0][0]=alpha;
    m[0][1]=beta;
    m[0][2]=(1-alpha)*center.x - beta*center.y;
    m[1][0]=-beta;
    m[1][1]=alpha;
    m[1][2]=beta*center.x + (1-alpha)* center.y;
    
    cvConvert(&M,matrix);
    
    return matrix;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	sensor_msgs::CvBridge bridge;
	try
	{
		last_img = bridge.imgMsgToCv(msg,"bgr8");
		if (count%sampling_rate==0)
		{
			//resize the image to fit exactly tablet screen
			CvSize size = cvSize(width,heigth);
			IplImage* tmp_img = cvCreateImage(size, last_img->depth, last_img->nChannels);
			cvResize(last_img, tmp_img);
			if (head_position<(-(MAX_HEAD_POS-MIN_HEAD_POS)/2))
                        { 
			    //printf("+++ rotating, head pos: %f\n",head_position);
			    IplImage* tmp_img_2 = cvCreateImage(size, last_img->depth, last_img->nChannels);
                            CvPoint2D32f pivot = cvPoint2D32f(half_width,half_heigth);
                            CvMat* rot_mat=cvCreateMat(2,3,CV_32FC1);
                            cv2DRotationMatrix(pivot,180,1,rot_mat);
                            cvWarpAffine(tmp_img,tmp_img_2,rot_mat);
			    sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img_2,"bgr8");
			    pub.publish(msg);
			    cvReleaseImage(&tmp_img_2);
                        }
                        else
			{
			    //printf("+++ normal, head pos: %f",head_position);
			    sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img,"bgr8");
			    pub.publish(msg);
			}
                        cvReleaseImage(&tmp_img);
			count=0;
		}
		/*else
			printf("+\n");*/
		count++;
	}
	catch(sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("cannot convert");
	}
}	

void head_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)//(const pr2_controllers_msgs::JointTrajectoryControllerState& msg)
{
	trajectory_msgs::JointTrajectory traj= *msg;
	double a=traj.points[0].positions[0];
	printf("new head position: %f\n",a);
	head_position=a;
}

int main(int argc, char** argv)
{
	//setting up default values
	width=DEFAULT_WIDTH;
	heigth=DEFAULT_HEIGTH;
	input_topic=DEFAULT_INPUT_TOPIC;
	output_topic=DEFAULT_OUTPUT_TOPIC;
	head_controller_topic=DEFAULT_HEAD_TOPIC;
	sampling_rate=RATE;
        //reading the configuration
	std::string path=ros::package::getPath("AccompanyImagesRepublisher")+"/config/config.txt";
	//std::cout << path << "\n";
	
	std::ifstream infile(path.c_str());

	std::string param,value;
	while(infile >> param >> value)
	{
		//std::cout << "param: " << param << ", value: " << value <<"\n";
		if (param=="image_width")
		{
			sscanf(value.c_str(), "%d", &width);
		}
		if (param=="image_heigth")
		{	
			sscanf(value.c_str(), "%d", &heigth);
		}
		if (param=="input_topic")
		{
			input_topic=value;
		}
		if (param=="output_topic")
		{
			output_topic=value;
		}
		if (param=="head_control_topic")
		{
			head_controller_topic=value;
		}
		if (param=="sampling_rate")
		{
			sscanf(value.c_str(), "%d", &sampling_rate);
		}
	}
	std::cout <<"Width:" <<width <<", Heigth:"<<heigth<<"\n";
	std::cout <<"Input topic: " <<input_topic<<"\n";
	std::cout <<"Output topic: " <<output_topic<<"\n";
	std::cout <<"Head Controller topic: " <<head_controller_topic<<"\n";
	std::cout <<"Sampling rate: "<<sampling_rate<<"\n";

	//computing (here -> only once!) pivot point coords:
	half_width=(int)width/2;
	half_heigth=(int)heigth/2;
	std::cout <<"HalfWidth:" <<half_width <<", HalfHeigth:"<<half_heigth<<"\n";
	
	ros::init(argc,argv,"Accompany_Republisher");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it2(nh);
	image_transport::Subscriber sub = it2.subscribe(input_topic, 1, image_callback);


	image_transport::ImageTransport it(nh);
	pub = it.advertise(output_topic,1);

	//Subscribing head position:
	ros::Subscriber head_sub=nh.subscribe(head_controller_topic,1,head_callback);
	
	std::cout <<"\n\n";
	std::cout << "Remapping topic \"" <<input_topic << "\" on \"" << output_topic <<"\"\n";
        std::cout << "Downsizing images to "<<width<<"x"<<heigth<<"\n";
	std::cout << "Cutting image frequency with "<<sampling_rate <<" ratio.\n";
	std::cout << "Images Republisher started...\n"	;

	ros::spin();
	/*ros::Rate loop_rate(0.2);
	while (nh.ok())
	{
		if (last_img!= NULL) 
		{
			printf("**\n");
			sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(last_img,"bgr8");
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}*/
 	//cvReleaseImage(&last_img);
        
}
