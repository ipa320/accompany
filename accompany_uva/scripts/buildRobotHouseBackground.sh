#!/bin/bash

camera1Path=`readlink -f ../res/testRobotHouse/camera1/`
camera2Path=`readlink -f ../res/testRobotHouse/camera2/`
camera3Path=`readlink -f ../res/testRobotHouse/camera3/`


mkdir -p $camera1Path/background_images
mkdir -p $camera2Path/background_images
mkdir -p $camera3Path/background_images

echo "build background model"
roslaunch accompany_static_camera_localisation capture_background_images.launch image_topic:=/camera1/gscam/image_raw nr_images:=10 res_path:=$camera1Path capture_name:="capture1"; roslaunch accompany_static_camera_localisation build_background_model.launch res_path:=$camera1Path &

roslaunch accompany_static_camera_localisation capture_background_images.launch image_topic:=/camera/rgb/image_rect_color nr_images:=10 res_path:=$camera2Path capture_name:="capture2"; roslaunch accompany_static_camera_localisation build_background_model.launch res_path:=$camera2Path &

roslaunch accompany_static_camera_localisation capture_background_images.launch image_topic:=/camera3/gscam/image_raw nr_images:=10 res_path:=$camera3Path capture_name:="capture3"; roslaunch accompany_static_camera_localisation build_background_model.launch res_path:=$camera3Path

ls -l $camera1Path/bgmodel.xml
ls -l $camera2Path/bgmodel.xml
ls -l $camera3Path/bgmodel.xml
