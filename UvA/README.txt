#                         #
# UNIVERSITY OF AMSTERDAM #
#           2012          #

# -------------------------
# ---  Introduction
# -------------------------

This document describes how to build and use the UvA modules of the
Accompany project. We assume installation on Ubuntu 11.10 (Oneiric).

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

  cd cmn
  mkdir build
  cd build
  cmake ../src
  make
  sudo make install  



# -------------------------
# ---  other stuff ...............
# -------------------------

Compile bnaic

  cd $ACCOMPANY_PATH/UvA/LikelihoodFromFisheyeCamera/bnaic
  mkdir build
  cd build
  cmake ../src -DVXL_BASE_DIR=/usr/local/include/vxl -DCMN_SRC_DIR=../../cmn/src -DCMN_LIB_DIR=../../cmn/build -DOPENCV_DIR=/usr/include/opencv-2.3.1/
  make
  
# ----------------------------------------
# --- Package StaticCameraLocalisation ---
# ----------------------------------------

= OVERVIEW OF AVAILABLE NODES
  
  - CameraLocalisation [main function to localize persons]
  
  - BuildBackgroundModel [build background model with PCA]
  
  - CreatePrior [select a region on the groundplane]
  
  - CalibrationExtrinsic [calibrate extrinsic parameters of overhead camera]
  
  - CalibrationIntrinsic [calibrate intrinsic parameters of overhead camera]
  
  
= TEST ROUTINE

 - Go to test folder:

    cd [accompany folder]/accompany_static_camera_localisation/test

 - Create a image list containing chessboard patterns:
    
    rosrun accompany_static_camera_localisation create_calibration_list calib_list.xml pattern_test/left*.jpg
    
 - Intrinsic calibration:

    rosrun accompany_static_camera_localisation calibration_intrinsic -w 6 -h 9 -u 1 -d 500 -o left_intrinsic.xml -i calib_list.xml
  
 - Extrinsic calibration:

    rosrun accompany_static_camera_localisation calibration_extrinsic -i camera_intrinsic.xml -o camera_extrinsic.xml -p points2D.txt -q points3D.txt
  
 - Create prior locations (select area that persons can walk on):

    rosrun accompany_static_camera_localisation create_prior -i imagelist_background.txt -p params.xml -o prior.txt
  
 - Build background model:

    rosrun accompany_static_camera_localisation build_background_model -i imagelist_background.txt -o bgmodel.xml
  
 - Camera Localization

    roscore
  
    *open another terminal*
  
    rosrun accompany_static_camera_localisation camera_localization bgmodel.xml params.xml prior.txt
  
    *open another terminal*
  
    rostopic echo /humanLocations

= TODO

 - Build a tracker
 
 - Adaptive background model
 
 - Multiple camera tracking
 
 - Final testing on frame stream
