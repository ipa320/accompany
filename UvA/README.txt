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


# ---------------------------------------------------
# ---  PACKAGE accompany_static_camera_localisation
# ---------------------------------------------------

#-- Test Routine --#

Download testing resource from:

  http://basterwijn.nl/ninghang/test_res/

We assume you are now in the same folder as the files you just downloaded. Load test video streams to ROS:

  rosrun accompany_static_camera_localisation video_publisher -s 0.3 -i wcam_20120112_vid4.avi 

Start localization:

  rosrun accompany_static_camera_localisation camera_localization bgmodel.xml params.xml prior.txt 
  
Show locations: 
  rostopic echo /humanLocations
  
----------------------------------------
  
  
#-- Preparation --#

Required: checkerboard with WHITE and LARGE boader, black or gray tape (more than 10m), tape measure.

Download checkerboard pattern from:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf
  
and print out checkerboard pattern on A1 paper, then attach the paper onto a board like:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
  
Use tape to make cross markers on the floor and also on the wall, with an interval of 1 meter. The markers represent the world coordinates frame. Write the coordinates of markers into a file, an example is:

  points3D.txt
  ----------
  0,0,0
  0,1000,0
  3000,0,1000
  ... 

Check camera manual

  focal length
  image
  
To import project into Eclipse (optional), refer to:
  
  http://www.ros.org/wiki/IDEs
  
----------------------------------------
#-- Intrinsic Calibration --#

Open camera in FULL resolution (HALF?)

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.0.10:8554/CH001.sdp ! decodebin ! videoscale ! ffmpegcolorspace"
  rosrun gscam gscam -s 0

Record a few frames containing a checkerboard:

  roscd accompany_static_camera_localisation/res/calib_frames
  rosrun accompany_static_camera_localisation image_saver image:=/gscam/image_raw
  
Filter out non-informative calibration frames in the folder, and then download the corner extracor:

  ./run_calib

Open MATLAB and extract corners using `run.m`, corner points will be saved in `X.csv` and `Y.csv`

Create a image list:
  
  rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml *.jpg
  cat calib_list.xml
    
Intrinsic calibration:

  rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 9 -o ../camera_intrinsic.xml -su calib_list.xml

----------------------------------------


#-- Intrinsic Calibration using ROS (low accuracy-DEPRECATED) --#

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
    
----------------------------------------


#-- Camera Extrinsic Calibration --#

Annotate marker locations in a HALF resolution frame:

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

----------------------------------------


#-- Build background model --#

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
  
----------------------------------------


#-- Create Prior --#

Select a walkable region:

  rosrun accompany_static_camera_localisation create_prior -l background/background_list.txt -p params.xml -o prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml
  
----------------------------------------


#-- Checkpoint calibration --#

  rosrun accompany_static_camera_localisation annotate_pos background/background_list.txt params.xml prior.txt x.txt camera_intrinsic.xml camera_extrinsic.xml

----------------------------------------

#-- Localization --#

  rosrun accompany_static_camera_localisation camera_localization -p [path]
  rostopic echo /humanLocations

----------------------------------------

= TODO
 
 - Adaptive background model
 
 - Multiple camera tracking
