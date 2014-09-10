/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: accompany
* \note
* ROS package name: accompany_object_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: 20.08.2014
*
* \brief
* functions for display of object detections
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/
// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <accompany_object_detection/object_detection.h>

#include <sstream>
#include <ctime>
#include <fstream>


class ObjectDetection
{
public:
	ObjectDetection(ros::NodeHandle nh) :
		node_handle_(nh)
	{
		it_ = 0;
		sync_input_ = 0;

		std::cout << "\n--------------------------\nObject Detection Parameters:\n--------------------------\n";
		node_handle_.param("object_detection/operation_mode", operation_mode_, 0);
		std::cout << "operation_mode = " << operation_mode_ << std::endl;

		it_ = new image_transport::ImageTransport(node_handle_);
		colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
		pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);
		sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
		sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);

		if (operation_mode_ == 2)
		{
			training_data_file_.open("accompany_object_detection/images/data.txt", std::ios_base::in);
			if (training_data_file_.is_open() == false)
			{
				std::cout << "Error: Could not open data annotation file." << std::endl;
				return;
			}
			trainObjects();
		}
		else if (operation_mode_ == 1)
		{
			training_data_file_.open("accompany_object_detection/images/data.txt", std::fstream::out | std::fstream::app);
			if (training_data_file_.is_open() == false)
			{
				std::cout << "Error: Could not open data annotation file." << std::endl;
				return;
			}
			sync_input_->registerCallback(boost::bind(&ObjectDetection::inputCallbackCapture, this, _1, _2));
		}
		else
		{
			mlp_.load("accompany_object_detection/classifier/nn.yaml");
			sync_input_->registerCallback(boost::bind(&ObjectDetection::inputCallbackDetection, this, _1, _2));
		}
	}

	~ObjectDetection()
	{
		if (it_ != 0)
			delete it_;
		if (sync_input_ != 0)
			delete sync_input_;

		if (training_data_file_.is_open() == true)
			training_data_file_.close();
	}

	// Converts a color image message to cv::Mat format.
	void convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
	{
		try
		{
			image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		}
		image = image_ptr->image;
	}


	void computeFeatures(const cv::Mat& image, cv::Mat& features, const cv::Size window_size=cv::Size(128,128), const cv::Size window_stride = cv::Size(8,8), const std::vector<cv::Point>& locations = std::vector<cv::Point>())
	{
		cv::HOGDescriptor hog;

		//selecting parameters for the HOG feature extractor. There are two feature extractor, one for each cell size (8x8, 16x16)
		hog.blockSize= cv::Size(16,16);
		hog.winSize= window_size;
		hog.blockStride=cv::Size(8,8);
		hog.cellSize= cv::Size(8,8);

//		cv::Mat image2, im;
//		cv::resize(image, im, cv::Size(RES_X,RES_Y));
//		//im.copyTo(image2, circle);
//		im.copyTo(image2);

		//Computing the histogram of oriented gradients features
		const cv::Size trainingPadding = cv::Size(0,0);
		std::vector<float> featureVector;
		hog.compute(image, featureVector, window_stride, trainingPadding, locations);
		//std::cout << "featureVector.size()=" << featureVector.size() << "   locations.size()=" << locations.size() << std::endl;

		if (locations.size() == 0)
			features.create(1, featureVector.size(), CV_32FC1);
		else
			features.create(locations.size(), featureVector.size()/locations.size(), CV_32FC1);

		unsigned int j=0;
		for (int r=0; r<features.rows; ++r)
			for(int c=0; c<features.cols; ++c, ++j)
				features.at<float>(r,c)= featureVector.at(j);
	}

	void inputCallbackDetection(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		//ROS_INFO("Input Callback Detection");

		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// get color image from point cloud
		// pcl::PointCloud < pcl::PointXYZRGB > point_cloud_src;
		// pcl::fromROSMsg(*pointcloud_msg, point_cloud_src);

		cv::Size window_size(128,128);
		cv::Size window_stride(16,16);
		std::vector<cv::Point> locations;
		for (int y=32; y<color_image.rows-window_size.height-32; y+=window_stride.height)
		{
			for (int x=192; x<color_image.cols-window_size.width-32; x+=window_stride.width)
			{
				locations.push_back(cv::Point(x,y));
			}
		}
//		locations.push_back(cv::Point(256,176));
//		cv::rectangle(color_image, cv::Rect(locations[0].x, locations[0].y, window_size.width, window_size.height), CV_RGB(255, 0, 0), 2);
		cv::Mat features;
		computeFeatures(color_image, features, window_size, window_stride, locations);

		cv::Mat responses;
		mlp_.predict(features, responses);
		//std::cout << "responses (r,c)=(" << responses.rows << ", " << responses.cols << ")" << std::endl;

		//std::cout << "This is class " << responses.at<float>(0,0) << std::endl;
		for (unsigned int i=0; i<locations.size(); ++i)
		{
			if (responses.at<float>(i,0) > 0.9)
			{
				std::cout << "Found object with response=" << responses.at<float>(i,0) << std::endl;
				cv::rectangle(color_image, cv::Rect(locations[i].x, locations[i].y, window_size.width, window_size.height), CV_RGB(0, 255, 0), 2);
			}
		}

//		int stride_x = 10;
//		int stride_y = 1000;
//		int roi_width = 130;
//		int roi_height = 160;
//		for (int y=160; y<color_image.rows-roi_height; y+=stride_y)
//		{
//			for (int x=0; x<color_image.cols-roi_width; x+=stride_x)
//			{
//				cv::Rect window(x, y, roi_width, roi_height);
//				cv::Mat roi = color_image(window);
//
//				cv::Mat features;
//				computeFeatures(roi, features);
//
//				cv::Mat response;
//				mlp_.predict(features, response);
//
//				//std::cout << "This is class " << response.at<float>(0,0) << std::endl;
//				if (response.at<float>(0,0) > 0.75)
//					cv::rectangle(color_image, window, CV_RGB(0, 255, 0), 2);
//			}
//		}

		cv::imshow("color_image", color_image);
		cv::waitKey(10);

		// cv::Mat color_image = cv::Mat::zeros(point_cloud_src.height, point_cloud_src.width, CV_8UC3);
		// for (unsigned int v=0; v<point_cloud_src.height; v++)
		// {
		// for (unsigned int u=0; u<point_cloud_src.width; u++)
		// {
		// pcl::PointXYZRGB point = point_cloud_src(u,v);
		// if (isnan_(point.z) == false)
		// color_image.at<cv::Point3_<unsigned char> >(v,u) = cv::Point3_<unsigned char>(point.b, point.g, point.r);
		// }
		// }
	}

	void trainObjects()
	{
		// compute feature data
		cv::Mat feature_mat;
		cv::Mat label_mat;
		while (training_data_file_.eof() == false)
		{
			int class_id = 0;
			std::string filename;
			training_data_file_ >> class_id >> filename;
			if (class_id == 0 && filename.length() == 0)
				break;

			std::cout << "Reading sample with class_id: " << class_id << "   filename: " << filename << std::endl;

			cv::Mat image = cv::imread(filename);

			cv::Mat label(1,1,CV_32FC1);
			label.at<float>(0,0) = (float)class_id;
			cv::Mat features;
			const int RES_X = 128;
			const int RES_Y = 128;
			cv::Mat image2;
			cv::resize(image, image2, cv::Size(RES_X,RES_Y));
			computeFeatures(image2, features);

//			cv::imshow("image", image);
//			cv::waitKey();

			feature_mat.push_back(features);
			label_mat.push_back(label);
		}

		std::cout << "feature_mat: " << feature_mat.rows << " " << feature_mat.cols << std::endl;
		std::cout << "label_mat: " << label_mat.rows << " " << label_mat.cols << std::endl;
//		for (int v=0; v<feature_mat.rows; ++v)
//		{
//			for (int u=0; u<feature_mat.cols; ++u)
//				std::cout << feature_mat.at<float>(v,u) << "\t";
//			std::cout << std::endl;
//		}

		// train classifier
		//	Neural Network
		cv::Mat input;
		feature_mat.convertTo(input, CV_32F);
		cv::Mat output=cv::Mat::zeros(feature_mat.rows, 1, CV_32FC1);
		cv::Mat labels;
		label_mat.convertTo(labels, CV_32F);
		for(int i=0; i<feature_mat.rows; ++i)
			output.at<float>(i,0) = labels.at<float>(i,0);

		cv::Mat layers = cv::Mat(3,1,CV_32SC1);

		layers.row(0) = cv::Scalar(feature_mat.cols);
		layers.row(1) = cv::Scalar(10);
		layers.row(2) = cv::Scalar(1);

		CvANN_MLP_TrainParams params;
		CvTermCriteria criteria;
		criteria.max_iter = 100;//100;
		criteria.epsilon  = 0.0001f; // farhadi:0.0001f, handcrafted:0.00001f;
		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

		params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
		params.bp_dw_scale     = 0.1f;
		params.bp_moment_scale = 0.1f;
		params.term_crit       = criteria;

		mlp_.create(layers,CvANN_MLP::SIGMOID_SYM, 0.4, 1.0);			// 0.4, except for dominant/sec. dom. color: 0.2
		int iterations = mlp_.train(input, output, cv::Mat(), cv::Mat(), params);
		std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;

		mlp_.save("accompany_object_detection/classifier/nn.yaml");
	}

	void inputCallbackCapture(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		//ROS_INFO("Input Callback Capture");

		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// get color image from point cloud
		pcl::PointCloud < pcl::PointXYZRGB > point_cloud_src;
		pcl::fromROSMsg(*pointcloud_msg, point_cloud_src);

		cv::Rect roi_rect(256, 176, 128, 128);
		cv::Mat display_image = color_image.clone();
		cv::rectangle(display_image, roi_rect, CV_RGB(255, 0, 0), 2);
		cv::imshow("color_image", display_image);
		int key = cv::waitKey(10);

		// capture image
		if (key == 'b' || key=='f' || key=='v')
		{
//			cv::Mat gray_image;
//			cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
//			double min_gray=0.0, max_gray=255.0;
//			cv::minMaxLoc(gray_image, &min_gray, &max_gray);
//			int white_threshold = max_gray - 0.4*(max_gray-min_gray);

//			// compute roi
//			int min_u = color_image.cols, max_u = 0;
//			int min_v = color_image.rows, max_v = 0;
//			for (int v=0; v<color_image.rows; ++v)
//			{
//				for (int u=0; u<color_image.cols; ++u)
//				{
//					//cv::Vec3b& pixel = color_image.at<cv::Vec3b>(v,u);
//					pcl::PointXYZRGB point = point_cloud_src(u,v);
//					//if (gray_image.at<unsigned char>(v,u) >= white_threshold)
//					if (point.z == point.z && point.z < 1.2)
//					{
//						if (min_u > u) min_u = u;
//						if (max_u < u) max_u = u;
//						if (min_v > v) min_v = v;
//						if (max_v < v) max_v = v;
//					}
//				}
//			}
//			if (max_u-min_u < 1 || max_v-min_v < 1)
//				return;

//			cv::Mat roi = color_image(cv::Rect(min_u, min_v, max_u-min_u, max_v-min_v));
			cv::Mat roi = color_image(roi_rect);

			// store to disk
			time_t tim;
			std::stringstream ss;
			ss << "image_" << time(&tim) << ".png";
			std::string filename = "accompany_object_detection/images/";
			if (key == 'b')
			{
				filename += "background/";
				training_data_file_ << 0 << "\t";
			}
			else if (key == 'f')
			{
				filename += "flowers/";
				training_data_file_ << 1 << "\t";
			}
			else if (key == 'v')
			{
				filename += "vase/";
				training_data_file_ << 2 << "\t";
			}
			filename += ss.str();
			training_data_file_ << filename << std::endl;

			cv::imwrite(filename, roi);

			cv::imshow("captured image", roi);
			cv::waitKey(10);

			ROS_INFO("Image captured and stored to %s.", filename.c_str());
		}
		else if (key=='q')
			exit(0);
	}

private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	std::fstream training_data_file_;

	CvANN_MLP mlp_;

	// parameters
	int operation_mode_;		// 0=detect, 1=capture training images, 2=train classifier
};


int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "accompany_object_detection");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	ObjectDetection objectDetection(nh);
	ros::spin();

	return (0);
}
