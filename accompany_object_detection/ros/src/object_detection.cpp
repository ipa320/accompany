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
			sync_input_->registerCallback(boost::bind(&ObjectDetection::inputCallbackTraining, this, _1, _2));
		else if (operation_mode_ == 1)
		{
			training_data_file_.open("accompany_object_detection/images/data.txt", std::ios::app);
			sync_input_->registerCallback(boost::bind(&ObjectDetection::inputCallbackCapture, this, _1, _2));
		}
		else
			sync_input_->registerCallback(boost::bind(&ObjectDetection::inputCallbackDetection, this, _1, _2));
	}

	~ObjectDetection()
	{
		if (it_ != 0)
			delete it_;
		if (sync_input_ != 0)
			delete sync_input_;

		if (operation_mode_ == 1)
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

	void inputCallbackDetection(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		ROS_INFO("Input Callback Detection");
		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// get color image from point cloud
		// pcl::PointCloud < pcl::PointXYZRGB > point_cloud_src;
		// pcl::fromROSMsg(*pointcloud_msg, point_cloud_src);

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

		cv::imshow("color_image", color_image);
		int key = cv::waitKey(10);

		// capture image
		if (key == 'b' || key=='f' || key=='v')
		{
//			cv::Mat gray_image;
//			cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
//			double min_gray=0.0, max_gray=255.0;
//			cv::minMaxLoc(gray_image, &min_gray, &max_gray);
//			int white_threshold = max_gray - 0.4*(max_gray-min_gray);

			// compute roi
			int min_u = color_image.cols, max_u = 0;
			int min_v = color_image.rows, max_v = 0;
			for (int v=0; v<color_image.rows; ++v)
			{
				for (int u=0; u<color_image.cols; ++u)
				{
					//cv::Vec3b& pixel = color_image.at<cv::Vec3b>(v,u);
					pcl::PointXYZRGB point = point_cloud_src(u,v);
					//if (gray_image.at<unsigned char>(v,u) >= white_threshold)
					if (point.z == point.z && point.z < 1.2)
					{
						if (min_u > u) min_u = u;
						if (max_u < u) max_u = u;
						if (min_v > v) min_v = v;
						if (max_v < v) max_v = v;
					}
				}
			}
			if (max_u-min_u < 1 || max_v-min_v < 1)
				return;

			cv::Mat roi = color_image(cv::Rect(min_u, min_v, max_u-min_u, max_v-min_v));

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

	void inputCallbackTraining(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		ROS_INFO("Input Callback Training");
	}

private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	std::ofstream training_data_file_;

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
