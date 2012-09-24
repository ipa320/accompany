#                         #
# UNIVERSITY OF AMSTERDAM #
#    ACCOMPANT PROJECT    #
#           2012          #
#                         #


# -------------------------
# ---  Introduction
# -------------------------

This document describes how to build and use the UvA modules (DoW T4.1) of the ACCOMPANY project. We assume installation on Ubuntu 11.10 (Oneiric).

# -------------------------
# ---  Dependencies
# -------------------------

Install ROS 'ros-electric-desktop-full' using instructions here:

  http://www.ros.org/wiki/electric/Installation

Download vxl-1.17.0 from

  http://sourceforge.net/projects/vxl/files/vxl/1.17/vxl-1.17.0.zip/download

and install using (this will take some time):

  unzip vxl-1.17.0.zip
  cd vxl-1.17.0/
  mkdir build
  cd build
  cmake .. -DBUILD_BRL=OFF
  make -j 4
  sudo make install

Install ubuntu-restricted-extras

  sudo apt-get -y install ubuntu-restricted-extras

Install other software requirements and test:

  ./installUvA.sh

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

  roscd accompany/UvA/testData
  rosrun accompany_static_camera_localisation video_publisher -s 0.3 -i wcam_20120112_vid4.avi 

Start localization:

  rosrun accompany_static_camera_localisation camera_localization -p.
  
Show locations:
  rostopic echo /humanLocations
  
----------------------------------------
  
  
#-- Preparation --#

Required: checkerboard with WHITE and LARGE boader, black or gray tape (> 10m), tape measure.

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

KINECT

  roscd accompany_static_camera_localisation/res/calib_frames
  ../../scripts/kinect_color_saver.sh
  
FISH-EYE

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:sadmin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"
  roscd accompany_static_camera_localisation/res/calib_frames
  
  rosrun gscam gscam -s 0
  roscd accompany_static_camera_localisation/res/calib_frames
  rosrun image_view image_view image:=/gscam/image_raw
  rosrun accompany_static_camera_localisation image_saver image:=/gscam/image_raw

Create a image list:
  
  rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml *.jpg
  cat calib_list.xml
    
Intrinsic calibration:

  rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 8 -o ../camera_intrinsic.xml -su calib_list.xml

Test:

  rosrun accompany_static_camera_localisation undistortion ../camera_intrinsic.xml [image name]

----------------------------------------


#-- [DEPRECATED_LOW ACCURACY]KINECT Intrinsic Calibration (using ROS) --#

Run calibration:

  roscd accompany_static_camera_localisation/res/
  ../scripts/kinect_calibration.sh

More information refers to refer to:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration

Open the intrinsic sample:

  gedit camera_intrinsic.xml &

Copy the calibration info from the command line to the file:

Capture a new frame:

  roscd accompany_static_camera_localisation/res/
  ../scripts/kinect_color.sh
  [RIGHT CLICK on the image]

Test the undistorted image

  rosrun accompany_static_camera_localisation undistortion camera_intrinsic.xml frame0000.jpg

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

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:sadmin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=768, height=729, framerate=15/1 ! ffmpegcolorspace"
  roscd accompany_static_camera_localisation/res/calib_frames
  rosrun gscam gscam -s 0

Capture a few background frames:

  roscd accompany_static_camera_localisation/res
  mkdir background_images
  cd background_images
  rosrun image_view image_view image:=/gscam/image_raw

Right click to store a few background frames

Check images

Process imaegs

Create a background image list:
  
  rosrun accompany_static_camera_localisation create_background_list background_list.txt *.jpg
  
Build background model:
  
  roscd accompany_static_camera_localisation/res
  rosrun accompany_static_camera_localisation build_background_model -i background/background_list.txt -o bgmodel.xml
  
----------------------------------------


#-- Create Prior --#

Select a walkable region:

  rosrun accompany_static_camera_localisation create_prior -l background_images/background_list.txt -p params.xml -o prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml
  
----------------------------------------


#-- Check calibration --#

  rosrun accompany_static_camera_localisation annotate_pos background_images/background_list.txt params.xml prior.txt x.txt camera_intrinsic.xml camera_extrinsic.xml

----------------------------------------

#-- Localization --#

  rosrun accompany_static_camera_localisation camera_localization -p [path]
  rostopic echo /humanLocations

----------------------------------------

= TODO
 
 - Adaptive background model
 
 - Multiple camera tracking
