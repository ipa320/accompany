

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
  
# --------------------------------------
# --- Package StaticCameraLocalisation
# --------------------------------------

Nodes Dependency
  
  - CameraLocalisation [main function to localize persons]
     |
     - CalibrationIntrinsic [calibrate intrinsic parameters of overhead camera]
     |
     - CalibrationExtrinsic [calibrate extrinsic parameters of overhead camera]
     |
     - CreatePrior [select a region on the groundplane]
     |
     - BuildBackgroundModel [build background model with PCA]















