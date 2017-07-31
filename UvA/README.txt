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

# -------------------------
# ---  Camera positions
# -------------------------

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

Calibrate Fish-eye camera in half resolution, Kinect in full resolution

Start KINECT

  roslaunch accompany_static_camera_localisation kinect_image_viewer.launch
  
Start FISH-EYE

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"
//  roslaunch accompany_static_camera_localisation fisheye_calib_image_saver.launch
  rosrun gscam gscam -s 0
  roscd accompany_static_camera_localisation/res/calib_frames
  rosrun image_view image_view image:=/gscam/image_raw
  rosrun accompany_static_camera_localisation image_saver -n 5000 -p ./ -t /gscam/image_raw

Remove similar checkerboard images
  ./modPics.sh
  mkdir OLD
  mv *jpg_OLD OLD/

Check checkerboard images if they are clear
  eog *.jpg

Create a image list:
  
  roscd accompany_static_camera_localisation/res/calib_frames
  rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml *.jpg
  cat calib_list.xml
    
Intrinsic calibration: (calibration improved by adding masks)

  rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 8 -m ../mask_large.png -k 5 -a 1 -rm -p -zt -o ../camera_intrinsic.xml calib_list.xml 



Test intrinsic calibration:

  - undistort single frame -

  roscd accompany_static_camera_localisation/res/calib_frames

  rosrun accompany_static_camera_localisation undistortion_test -s [image] -i [camera_intrinsic] -f


  - show live stream -

  restart gscam node with

    roscd accompany_static_camera_localisation/res

    rosrun gscam gscam -s 0 -i camera_intrinsic.xml

  generate undistorted stream

    ROS_NAMESPACE=/gscam rosrun image_proc image_proc 

  view live stream

    rosrun image_view image_view image:=/gscam/image_rect_color
    

----------------------------------------

#-- [DEPRECATED_DUE_TO_UNCONTROLED_IMAGE_STREAM] Intrinsic Calibration (using ROS) --#

More information refers to refer to:

  http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration

----------------------------------------

#-- Camera Extrinsic Calibration --#

FISH-EYE: Annotate marker locations in a HALF resolution frame:

  roscd accompany_static_camera_localisation/scripts
  ./fisheye_marker_images.sh

KINECT:

  roscd accompany_static_camera_localisation/scripts
  ./kinect_marker_images.sh

Right click to save a frame

Create a image list of the marker:
  
  roscd accompany_static_camera_localisation/res/marker/
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

#-- Build background model --# (adaptive background integrated, no need for this part any more)

roscd accompany_static_camera_localisation/scripts/

FISHEYE:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

  ./fisheye_capture_background_images.sh

Right click images to store background

KINECT:

  ./kinect_capture_background_images.sh

Both:

  ./create_background_model.sh
  
----------------------------------------

#-- Create Prior --#

Select a walkable region:

  roscd accompany_static_camera_localisation/res
  

  rosrun accompany_static_camera_localisation create_prior -l ./image_list.txt -p ./params.xml

  columns of image_list.txt are camera index

  #rosrun accompany_static_camera_localisation create_prior -l ./marker_list.txt -p ./params.xml
  #rosrun accompany_static_camera_localisation create_prior -l background_images/background_list.txt -p params.xml -o prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml
  
---------------------------------------

#-- Create Entrance Area --#

NOTE!!
image_list.txt has filename of images from multiple cameras on one line sperated by whitespace
temperarily set SCALE in params.xml to 1

rosrun accompany_static_camera_localisation create_entry_exit_areas -l ./image_list.txt -p ./params.xml


#-- Create --#
Create mapping between camera room coordinates and care-o-bot
       
       rosrun accompany_static_camera_localisation create_tf_room2world -m ./map.pgm -p ./map.yaml -n room_frame


----------------------------------------

#-- Check calibration --#

Check the calibration results:

  roscd accompany_static_camera_localisation/res

  rosrun accompany_static_camera_localisation annotate_pos -l background_images/background_list.txt -p params.xml -r prior.txt -i camera_intrinsic.xml -e camera_extrinsic.xml -a temp.txt

----------------------------------------

#-- Localization --#

FISHEYE:

  export GSCAM_CONFIG="rtspsrc location=rtsp://admin:admin@192.168.111.10:8554/CH001.sdp ! decodebin ! videoscale ! videorate ! video/x-raw-yuv, width=1024, height=972, framerate=15/1 ! ffmpegcolorspace"

  roslaunch accompany_static_camera_localisation fisheye_localization.launch

KINECT:
  roslaunch openni_launch openni.launch   
  rosrun accompany_static_camera_localisation camera_localization -p ./ image:=/camera/rgb/image_color -v -n 1

Echo human localizations:

  rostopic echo /humanLocations

----------------------------------------

script ./fast_start.sh

----------------------------------------

= TODO
 
 - Adaptive background model
 
 - Multiple camera tracking
 
-----------------------------------------

OpenNI skeleton tracking on ROS Fuerte + Ubuntu 12.04

In the default openni-dev package, the skeleton tracker binaries are out-of-date. To make it work, download OpenNI Compliant Middleware Binaries and do an uninstall and then install. This will replace/install some libraies and make it work
