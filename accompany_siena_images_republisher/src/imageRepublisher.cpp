#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

namespace enc = sensor_msgs::image_encodings;

//head constants
#define MAX_HEAD_POS 0.0001
#define MIN_HEAD_POS -3.1417

#define COB_3_2 0
#define COB_3_6 1

//image rate constants
#define RATE 4        // 3 limit number on my netWork in Siena

//image size default values
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGTH 752

//default topics
#define DEFAULT_OUTPUT_TOPIC "accompany/GUIimage"
#define DEFAULT_INPUT_TOPIC "/stereo/left/image_color"
#define DEFAULT_HEAD_TOPIC "head_controller/command"

//Cob version (defines the rotation)
int cob_version= COB_3_6;

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
	//sensor_msgs::CvBridge bridge;
	try
	{
		//last_img = bridge.imgMsgToCv(msg,"bgr8");
		cv_bridge::CvImagePtr ppp = cv_bridge::toCvCopy(msg, enc::BGR8);
		IplImage temp = ppp->image;		
		last_img = &temp;
		if (count%sampling_rate==0)
		{
			//resize the image to fit exactly tablet screen
			CvSize size = cvSize(width,heigth);
			IplImage* tmp_img = cvCreateImage(size, last_img->depth, last_img->nChannels);
			cvResize(last_img, tmp_img);
			if (head_position>(-(MAX_HEAD_POS-MIN_HEAD_POS)/2))//<
                        { 
			    if (cob_version==COB_3_6)
			    {
				    //printf("+++ rotating, head pos: %f\n",head_position);
				    IplImage* tmp_img_2 = cvCreateImage(size, last_img->depth, last_img->nChannels);
		                    CvPoint2D32f pivot = cvPoint2D32f(half_width,half_heigth);
		                    CvMat* rot_mat=cvCreateMat(2,3,CV_32FC1);
		                    cv2DRotationMatrix(pivot,180,1,rot_mat);
		                    cvWarpAffine(tmp_img,tmp_img_2,rot_mat);
				    //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img_2,"bgr8");
				    //pub.publish(msg);

				    ppp->image=tmp_img_2;
				    pub.publish(ppp->toImageMsg());

				    cvReleaseImage(&tmp_img_2);
			    }
			    else
			    {
				    //printf("+++ normal, head pos: %f",head_position);
				    //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img,"bgr8");
				    //pub.publish(msg);
	  			    ppp->image=tmp_img;
				    pub.publish(ppp->toImageMsg());
			    }
                        }
                        else
			{
				if (cob_version==COB_3_6)
	                        {
			    		//printf("+++ normal, head pos: %f",head_position);
			    		//sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img,"bgr8");
			    		//pub.publish(msg);
  			    		ppp->image=tmp_img;
			    		pub.publish(ppp->toImageMsg());
				}
				else
				{
				    //printf("+++ rotating, head pos: %f\n",head_position);
				    IplImage* tmp_img_2 = cvCreateImage(size, last_img->depth, last_img->nChannels);
		                    CvPoint2D32f pivot = cvPoint2D32f(half_width,half_heigth);
		                    CvMat* rot_mat=cvCreateMat(2,3,CV_32FC1);
		                    cv2DRotationMatrix(pivot,180,1,rot_mat);
		                    cvWarpAffine(tmp_img,tmp_img_2,rot_mat);
				    //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(tmp_img_2,"bgr8");
				    //pub.publish(msg);

				    ppp->image=tmp_img_2;
				    pub.publish(ppp->toImageMsg());

				    cvReleaseImage(&tmp_img_2);
				}
			}
                        cvReleaseImage(&tmp_img);
			count=0;
		}
		/*else
			printf("+\n");*/
		count++;
	}//catch(sensor_msgs::CvBridgeException& e)
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cannot convert");
	}
}	

void head_callback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
	pr2_controllers_msgs::JointTrajectoryControllerState traj= *msg;
	double a=traj.actual.positions[0];
	//printf("new head position: %f\n",a);
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
	std::string path=ros::package::getPath("accompany_siena_images_republisher")+"/config/config.txt";
	//std::cout << path << "\n";
	
	std::ifstream infile(path.c_str());

	std::string param,value;
	while(infile >> param >> value)
	{
		//std::cout << "param: " << param << ", value: " << value <<"\n";
		if (param=="cob_version")
		{
			/*const char * c_val=value.c_str();
			if (strcmp(c_val,"cob3-6")==0||strcmp(c_val,"cob_3_6")==0||strcmp(c_val,"cob3_6")==0||strcmp(c_val,"cob-3-6")==0) cob_version==COB_3_6;
			if (strcmp(c_val,"cob3-2")==0||strcmp(c_val,"cob_3_2")==0||strcmp(c_val,"cob3_2")==0||strcmp(c_val,"cob-3-2")==0) cob_version==COB_3_2;
			printf("tmp cob version: %s\n",c_val);*/
			if (value=="cob3-6"||value=="cob_3_6"||value=="cob3-6"||value=="cob-3-6") cob_version=COB_3_6;
			if (value=="cob3-2"||value=="cob_3_2"||value=="cob3-2"||value=="cob-3-2") cob_version=COB_3_2;
		}
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
		if (param=="head_downsampled_control_topic")
		{
			head_controller_topic=value;
		}
		if (param=="sampling_rate")
		{
			sscanf(value.c_str(), "%d", &sampling_rate);
		}
	}
	if (cob_version== COB_3_6)		
		std::cout <<"Cob version: cob 3-6\n";
	else
		std::cout <<"Cob version: cob 3-2\n";
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

	//try to get robot version from environment
	try{
		std::cout << "Running robot: " << getenv("ROBOT") << "\n";
		std::string cob_run = getenv("ROBOT");
		if (cob_run=="cob3-2") cob_version=COB_3_2;
		if (cob_run=="cob3-6") cob_version=COB_3_6;
	}catch(const std::exception e)
	{
		std::cout << "Exception retriving running robot version: " << e.what() << "\n"; 
	}
	catch(...)
	{
		std::cout << "Generic exception retriving runninbg robot version: unknown.\n";
	}

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
