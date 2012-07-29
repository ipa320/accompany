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

# now search for required headers in usual places instead of relying on set paths above

# check cmn headers
find_path(CMN_INCLUDE_DIR cmn/random.hh
  HINTS  /usr/include/cmn /usr/local/include/cmn ${CMN_SRC_DIR}
  PATH_SUFFIXES cmn) 
if (NOT CMN_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find 'cmn/random.hh', please install cmn from ssh://basterwijn.nl/home/bterwijn/git/cmnGwenn.git or set CMN_SRC_DIR")
else (NOT CMN_INCLUDE_DIR)
  include_directories(${CMN_INCLUDE_DIR})
endif (NOT CMN_INCLUDE_DIR)

# check vxl headers
find_path(VNL_INCLUDE_DIR vnl/vnl_vector.h
  HINTS  /usr/include/vxl/core /usr/local/include/vxl/core ${VXL_BASE_DIR}/core
  PATH_SUFFIXES vnl) 
if (NOT VNL_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find 'vnl/vnl_vector.h', please install 'libvxl-dev' or set VXL_BASE_DIR")
else (NOT VNL_INCLUDE_DIR)
  include_directories(${VNL_INCLUDE_DIR})
endif (NOT VNL_INCLUDE_DIR)

# check vcl headers
find_path(VCL_INCLUDE_DIR vcl/vcl_iosfwd.h
  HINTS  /usr/include/vxl /usr/local/include/vxl ${VXL_BASE_DIR}
  PATH_SUFFIXES vcl) 
if (NOT VCL_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find 'vcl/vcl_iosfwd.h', please install 'libvxl-dev' or set VXL_BASE_DIR")
else (NOT VCL_INCLUDE_DIR)
  include_directories(${VCL_INCLUDE_DIR} ${VCL_INCLUDE_DIR}/vcl)
endif (NOT VCL_INCLUDE_DIR)


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} /usr/include/opencv ${VXL_BASE_DIR}/core ${VXL_BASE_DIR} ${VXL_BASE_DIR}/vcl )
LINK_DIRECTORIES(${VXL_BASE_DIR}/lib ${CMN_LIB_DIR})

ADD_LIBRARY( helpers
  ${PROJECT_SOURCE_DIR}/src/Helpers.cc
  ${PROJECT_SOURCE_DIR}/src/Background.cc
  ${PROJECT_SOURCE_DIR}/src/ImgProducer.cc
  ${PROJECT_SOURCE_DIR}/src/CamCalib.cc
  )

ADD_LIBRARY( calib
  ${PROJECT_SOURCE_DIR}/src/cameraModel.cpp
  ${PROJECT_SOURCE_DIR}/src/xmlUtil.cpp
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
rosbuild_add_executable(calibration_intrinsic src/calibration_intrinsic.cpp)
TARGET_LINK_LIBRARIES(calibration_intrinsic opencv_core opencv_calib3d opencv_imgproc opencv_highgui opencv_features2d boost_program_options)

rosbuild_add_executable(calibration_extrinsic src/calibration_extrinsic.cpp)
TARGET_LINK_LIBRARIES(calibration_extrinsic opencv_core opencv_calib3d opencv_imgproc opencv_highgui boost_program_options)

rosbuild_add_executable(build_background_model src/build_background_model.cpp)
TARGET_LINK_LIBRARIES(build_background_model helpers calib opencv_calib3d opencv_highgui opencv_core opencv_imgproc cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt boost_program_options ${XML2_LDFLAGS})

rosbuild_add_executable(create_prior src/create_prior.cpp)
TARGET_LINK_LIBRARIES( create_prior helpers calib opencv_calib3d opencv_highgui opencv_core opencv_imgproc cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt boost_program_options ${XML2_LDFLAGS})

rosbuild_add_executable(camera_localization src/camera_localization.cpp)
target_link_libraries(camera_localization helpers calib opencv_calib3d opencv_highgui opencv_core opencv_imgproc cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt boost_program_options ${XML2_LDFLAGS})

rosbuild_add_executable(create_calibration_list src/create_calibration_list.cpp)
target_link_libraries(create_calibration_list opencv_highgui opencv_core opencv_imgproc)

rosbuild_add_executable(create_background_list src/create_background_list.cpp)
target_link_libraries(create_background_list opencv_highgui opencv_core opencv_imgproc)

rosbuild_add_executable(annotate_image_points src/annotate_image_points.cpp)
target_link_libraries(annotate_image_points opencv_highgui opencv_core opencv_imgproc helpers calib cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt ${XML2_LDFLAGS})

rosbuild_add_executable( annotate_pos src/annotate_pos.cpp )
target_link_libraries( annotate_pos helpers calib opencv_calib3d opencv_highgui opencv_core opencv_imgproc cmn tools iberty vnl_algo vnl v3p_netlib boost_date_time-mt ${XML2_LDFLAGS})

rosbuild_add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver opencv_highgui opencv_core opencv_imgproc)
