cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE(FindPkgConfig)
pkg_check_modules(XML2 REQUIRED libxml-2.0)
SET( CMAKE_CXX_FLAGS ${XML2_CFLAGS})
SET(HOME "/home/ninghang")
SET( VXL_BASE_DIR /usr/include/vxl CACHE PATH "Location of the VXL library source code")
SET( CMN_SRC_DIR ${HOME}/Software/PeopleDetectionGwenn/cmn/src CACHE PATH "Location of the CMN library source code")
SET( CMN_LIB_DIR ${HOME}/Software/PeopleDetectionGwenn/cmn_build CACHE PATH "Location of the CMN compiled library")
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} /usr/include/opencv ${VXL_BASE_DIR}/core ${VXL_BASE_DIR} ${VXL_BASE_DIR}/vcl ${CMN_SRC_DIR})
LINK_DIRECTORIES(${VXL_BASE_DIR}/lib ${CMN_LIB_DIR})

ADD_LIBRARY( helpers
  ${PROJECT_SOURCE_DIR}/src/Helpers.cc
  ${PROJECT_SOURCE_DIR}/src/Background.cc
  ${PROJECT_SOURCE_DIR}/src/ImgProducer.cc
  ${PROJECT_SOURCE_DIR}/src/CamCalib.cc
  )

ADD_LIBRARY( calib
  ${PROJECT_SOURCE_DIR}/src/hu/cameraModel.cpp
  ${PROJECT_SOURCE_DIR}/src/hu/xmlUtil.cpp
  )

SET( OPENCV_INCLUDE_DIR /usr/local/include)
SET( OPENCV_LIB_DIR /usr/local/lib)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_DIRECTORIES(${OPENCV_LIB_DIR})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rosbuild
rosbuild_add_executable(CalibrationIntrinsic src/CalibrationIntrinsic.cpp)
TARGET_LINK_LIBRARIES(CalibrationIntrinsic opencv_core opencv_calib3d opencv_imgproc opencv_highgui opencv_features2d boost_program_options)

rosbuild_add_executable(CalibrationExtrinsic src/CalibrationExtrinsic.cpp)
TARGET_LINK_LIBRARIES(CalibrationExtrinsic opencv_core opencv_calib3d opencv_imgproc opencv_highgui boost_program_options)

rosbuild_add_executable(BuildBackgroundModel src/BuildBackgroundModel.cpp)

rosbuild_add_executable(CreatePrior src/CreatePrior.cpp)
TARGET_LINK_LIBRARIES( CreatePrior helpers calib opencv_calib3d opencv_highgui opencv_core opencv_imgproc cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt boost_program_options ${XML2_LDFLAGS})

rosbuild_add_executable(CameraLocalisation src/CameraLocalisation.cpp)
target_link_libraries(CameraLocalisation boost_program_options)
