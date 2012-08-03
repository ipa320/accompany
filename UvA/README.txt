#                         #
# UNIVERSITY OF AMSTERDAM #
#    ACCOMPANT PROJECT    #
#           2012          #
#                         #


# -------------------------
# ---  Introduction
# -------------------------

This document describes how to build and use the UvA modules (DoW T4.1) of the ACCOMPANY project. We assume installation on Ubuntu 11.10 (Oneiric).

We assume ACCOMPANY_PATH points to the root of the accompany
directory.

# -------------------------
# ---  Dependencies
# -------------------------

Install ROS 'ros-electric-desktop-full' using instructions here:

  http://www.ros.org/wiki/electric/Installation

Install OpenCV

  sudo apt-get install libopencv2.3-dev

Install CMAKE

  sudo apt-get install cmake

Download vxl-1.17.0 from

  http://sourceforge.net/projects/vxl/files/vxl/1.17/vxl-1.17.0.zip/download

and install using:

  unzip vxl-1.17.0.zip
  cd vxl-1.17.0/
  cmake .. -DBUILD_BRL=OFF
  make
  sudo make install

Clone cmn with (non-public, this requires ssh access to server!!)

  git clone ssh://basterwijn.nl/home/bterwijn/git/cmnGwenn 

and install with

  cd cmnGwenn
  mkdir build
  cd build
  cmake ../src
  make
  sudo make install

Install gstreamer

  sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev

Install the gscam ROS package in: UvA/dependencies/gscam
This file has been slightly altered to drop old frames (sync=false)
and so always provide the last frame.

----------------------------------------
Some examples of using gstreamer on a GeoVision GV-FE421 IP camera at 192.168.0.10:

display stream:

  gst-launch rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=640, height=480, framerate=15/1 ! xvimagesink sync=false

save stream to file:

  gst-launch rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=640, height=480, framerate=15/1 ! jpegenc ! avimux ! filesink location=video.avi

publish stream in ros:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=640, height=480, framerate=15/1 ! ffmpegcolorspace"
  rosrun gscam gscam --sync false
----------------------------------------



# --------------------------------------------
# ---  Package:
# ---  accompany_static_camera_localisation
# --------------------------------------------


Nodes Overview:
  
  - camera_localisation [main function to localize persons]
  
  - build_background_model [build background model with PCA]
  
  - create_prior [select a region on the groundplane]
  
  - calibration_extrinsic [calibrate extrinsic parameters of overhead camera]
  
  - calibration_intrinsic [calibrate intrinsic parameters of overhead camera]
  
  - create_calibration_list [create a list of images for calibration]

  - create_background_list [create a list of background images]
  
  - annotate_image_points [annotate points on the image space]

  - image_saver [save a bunch of image frames]

  - annotate_pos [visualize the calibration results]
  
# -------------------------
# ---  Test Routine
# -------------------------

Create a image list containing chessboard patterns:
  
  roscd /accompany_static_camera_localisation/test
  rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml pattern_test/left*.jpg
    
Intrinsic calibration:

  rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 9 -u 1 -d 500 -o left_intrinsic.xml -i calib_list.xml
  
Extrinsic calibration:

  rosrun accompany_static_camera_localisation calibration_extrinsic -i camera_intrinsic.xml -o camera_extrinsic.xml -p points2D.txt -q points3D.txt
  
Create prior locations (select area that persons can walk on):

  rosrun accompany_static_camera_localisation create_prior -i imagelist_background.txt -p params.xml -o prior.txt
  
Build background model:

  rosrun accompany_static_camera_localisation build_background_model -i imagelist_background.txt -o bgmodel.xml
  
Camera Localization

  rosrun accompany_static_camera_localisation camera_localization bgmodel.xml params.xml prior.txt  
  rostopic echo /humanLocations


# -------------------------
# ---  Preparation
# -------------------------

Required: checkerboard with WHITE and LARGE boader, black or gray tape (more than 10m), tape measure.

Download checkerboard pattern from:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf
  
and print out checkerboard pattern on A1 paper, then attach the paper onto a board like:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
  
Use tape to make cross markers on the floor and also on the wall, with an interval of 1 meter. The markers represent the world coordinates frame. Write the coordinates of markers into a file, an example is:

  points3D.txt
  ------------
  0,0,0
  0,1000,0
  3000,0,1000
  ... 

Check camera manual

  focal length
  image

# -----------------------------------
# ---  Intrinsic Calibration
# -----------------------------------

Open camera in FULL resolution

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! ffmpegcolorspace"
  rosrun gscam gscam -s 0

Record a few frames containing a checkerboard:

  roscd accompany_static_camera_localisation/res/
  mkdir calib_frames
  cd calib_frames
  rosrun accompany_static_camera_localisation image_saver image:=/gscam/image_raw
  
Filter out non-informative calibration frames in the folder

Create a image list:
  
  roscd  accompany_static_camera_localisation/res/
  rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml calib_frames/*.*g
    
Intrinsic calibration:

  rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 9 -u 1 -d 500 -i calib_list.xml -o camera_intrinsic.xml 

# ------------------------------------------------
# ---  Intrinsic Calibration (alternative)
# ------------------------------------------------

Set gscam to capture frames with FULL resolution and default frame rate:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! ffmpegcolorspace"
  rosrun gscam gscam -s 0 
  
Run calibration:

  rosrun camera_calibration cameracalibrator.py --size 9x6 --square 1 image:=/gscam/image_raw camera:=/gscam
  
For calibration, refer to:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration

Save calibrated display with extension ".ini":

  roscd accompany_static_camera_localisation
  mkdir res
  cd res
  gedit calib_intrinsic.ini
  [copy diplayed messages] 

Copy info in .ini to .xml, remove comma
  
# -----------------------------------
# ---  Camera Extrinsic Calibration
# ----------------------------------- 

Annotate marker locations in a full resolution frame:

  roscd accompany_static_camera_localisation/res/
  mkdir marker
  cd marker
  rosrun image_view image_view image:=/gscam/image_raw
  
Right click to save a frame, make sure all markers are present

Create a image list of the marker:
  
  rosrun accompany_static_camera_localisation create_background_list marker_list.txt *.jpg

Annotate corresponding 2D points on video frames:
 
  roscd accompany_static_camera_localisation/res  
  rosrun accompany_static_camera_localisation annotate_image_points marker/marker_list.txt points2D.txt
  [NOTE: press ENTER to save ]

Copy points3D.txt to res folder:
 
  cp [location]/points3D.txt .

Double check if points2D and points3D are correct

Calibrate extrinsic parameters:

  rosrun accompany_static_camera_localisation calibration_extrinsic -i camera_intrinsic.xml -o camera_extrinsic.xml -p points2D.txt -q points3D.txt

Modify param.xml and set SCALE according to the desired resolution

# -----------------------------------
# ---  Build background model
# -----------------------------------

Restart gscam and capture background images with REDUCED resolution:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=512, height=486 , framerate=15/1 ! ffmpegcolorspace"
  rosrun gscam gscam -s 0

Capture a few background frames:

  roscd accompany_static_camera_localisation/res
  mkdir background
  cd background
  rosrun image_view image_view image:=/gscam/image_raw

Right click to store a few background frames

Create a background image list:
  
  rosrun accompany_static_camera_localisation create_background_list background_list.txt *.jpg
  
Build background model:
  
  roscd accompany_static_camera_localisation/res
  rosrun accompany_static_camera_localisation build_background_model -i background/background_list.txt -o bgmodel.xml
  
# -----------------------------------
# ---  Create Prior
# -----------------------------------

Select a walkable region:

  rosrun accompany_static_camera_localisation create_prior -i background/background_list.txt -p params.xml -o prior.txt

# -----------------------------------
# ---  Checkpoint calibration
# -----------------------------------

Check calibration results:

  rosrun accompany_static_camera_localisation annotate_pos background/background_list.txt  params.xml prior.txt x.txt

# -----------------------------------
# ---  Localization
# -----------------------------------

  rosrun accompany_static_camera_localisation camera_localization bgmodel.xml params.xml prior.txt  
  rostopic echo /humanLocations

# -----------------------------------
# ---  For Eclipse Users
# -----------------------------------


= TODO
 
 - feed with live frames

 - Build a tracker
 
 - Adaptive background model
 
 - Multiple camera tracking
 
 - Final testing on frame stream
 
 - Need to release image in localization?
 
 - check all pointers
