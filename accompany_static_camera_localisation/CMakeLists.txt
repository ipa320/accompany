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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} /usr/include/opencv)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/accompany_static_camera_localisation)
# now search for required headers in usual places instead of relying on set paths above

# check cmn headers
find_path(CMN_INCLUDE_DIR cmn/random.hh
  HINTS  /usr/include/cmn /usr/local/include/cmn
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
  HINTS  /usr/include/vxl /usr/local/include/vxl
  PATH_SUFFIXES vcl)
if (NOT VCL_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find 'vcl/vcl_iosfwd.h', please install 'libvxl-dev' or set VXL_BASE_DIR")
else (NOT VCL_INCLUDE_DIR)
  include_directories(${VCL_INCLUDE_DIR} ${VCL_INCLUDE_DIR}/vcl)
endif (NOT VCL_INCLUDE_DIR)


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} /usr/include/opencv ${VXL_BASE_DIR}/core ${VXL_BASE_DIR} ${VXL_BASE_DIR}/vcl )
LINK_DIRECTORIES(${VXL_BASE_DIR}/lib ${CMN_LIB_DIR})

rosbuild_add_library( hull
  include/accompany_static_camera_localisation/Hull.h
  ${PROJECT_SOURCE_DIR}/src/Hull.cpp
  )

ADD_LIBRARY( staticCam
  ${PROJECT_SOURCE_DIR}/src//Helpers.hh
  ${PROJECT_SOURCE_DIR}/src/Helpers.cc
  ${PROJECT_SOURCE_DIR}/src/Background.hh
  ${PROJECT_SOURCE_DIR}/src/Background.cc
  ${PROJECT_SOURCE_DIR}/src/ImgProducer.hh
  ${PROJECT_SOURCE_DIR}/src/ImgProducer.cc
  ${PROJECT_SOURCE_DIR}/src/CamCalib.hh
  ${PROJECT_SOURCE_DIR}/src/CamCalib.cc
  ${PROJECT_SOURCE_DIR}/src/ImageMask.h
  ${PROJECT_SOURCE_DIR}/src/ImageMask.cpp
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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

SET( MY_STANDARD_LIBS 
  staticCam
  hull
  calib
  boost_program_options 
  boost_system 
  boost_filesystem
  boost_thread
  opencv_core
  opencv_highgui
  opencv_features2d
  opencv_calib3d
  opencv_imgproc
  cmn 
  tools 
  iberty 
  vnl_algo 
  vnl 
  v3p_netlib 
  boost_date_time-mt 
  ${XML2_LDFLAGS}
  MsgToMarkerArray
  # yaml-cpp # *!*!*!* causes errors in readCalibrationYml() *!*!*!*
  )

# rosbuild
rosbuild_add_executable(calibration_intrinsic src/calibration_intrinsic.cpp)
TARGET_LINK_LIBRARIES(calibration_intrinsic ${MY_STANDARD_LIBS})

rosbuild_add_executable(calibration_extrinsic src/calibration_extrinsic.cpp)
TARGET_LINK_LIBRARIES(calibration_extrinsic ${MY_STANDARD_LIBS})

rosbuild_add_executable(build_background_model src/build_background_model.cpp)
TARGET_LINK_LIBRARIES(build_background_model ${MY_STANDARD_LIBS} )

rosbuild_add_executable(create_prior src/create_prior.cpp)
TARGET_LINK_LIBRARIES( create_prior ${MY_STANDARD_LIBS})

rosbuild_add_executable(create_entry_exit_areas src/create_entry_exit_areas.cpp)
TARGET_LINK_LIBRARIES(create_entry_exit_areas ${MY_STANDARD_LIBS})

rosbuild_add_executable(camera_localization 
  src/camera_localization.cpp
  src/Histogram.h
  src/AppearanceExtractor.h
  src/AppearanceExtractor.cpp
 )
target_link_libraries(camera_localization ${MY_STANDARD_LIBS})

rosbuild_add_executable(create_calibration_list src/create_calibration_list.cpp)
target_link_libraries(create_calibration_list ${MY_STANDARD_LIBS})

rosbuild_add_executable(create_background_list src/create_background_list.cpp)
target_link_libraries(create_background_list ${MY_STANDARD_LIBS})

rosbuild_add_executable(annotate_image_points src/annotate_image_points.cpp)
target_link_libraries(annotate_image_points ${MY_STANDARD_LIBS} )

rosbuild_add_executable( annotate_pos src/annotate_pos.cpp )
target_link_libraries( annotate_pos ${MY_STANDARD_LIBS} )

rosbuild_add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver ${MY_STANDARD_LIBS} )

rosbuild_add_executable(video_publisher src/video_publisher.cpp)
target_link_libraries(video_publisher ${MY_STANDARD_LIBS} )

rosbuild_add_executable(create_frame src/create_frame.cpp)
target_link_libraries(create_frame ${MY_STANDARD_LIBS} )

rosbuild_add_executable(create_tf_room2world src/create_tf_room2world.cpp)
target_link_libraries(create_tf_room2world ${MY_STANDARD_LIBS} yaml-cpp )

rosbuild_add_executable(undistortion_test src/undistortion_test.cpp)
target_link_libraries(undistortion_test ${MY_STANDARD_LIBS} )

rosbuild_add_executable(testCameraModel src/testCameraModel.cpp)
target_link_libraries(testCameraModel ${MY_STANDARD_LIBS})

rosbuild_add_executable(test_transform src/test_transform.cpp)
target_link_libraries(test_transform ${MY_STANDARD_LIBS})

rosbuild_add_executable(resize_image src/resize_image.cpp)
target_link_libraries(resize_image ${MY_STANDARD_LIBS})

rosbuild_add_executable(add_camera_info src/add_camera_info.cpp)
target_link_libraries(add_camera_info ${MY_STANDARD_LIBS})

rosbuild_add_executable(publish_image src/publish_image.cpp)
target_link_libraries(publish_image ${MY_STANDARD_LIBS})

rosbuild_add_executable(kinect_driver src/kinect_driver.cpp)
target_link_libraries(kinect_driver ${MY_STANDARD_LIBS})

rosbuild_add_executable(publish_tf_from_extrinsic src/publish_tf_from_extrinsic.cpp)
target_link_libraries(publish_tf_from_extrinsic ${MY_STANDARD_LIBS})

rosbuild_add_executable(project_tf_to_camera src/project_tf_to_camera.cpp)
target_link_libraries(project_tf_to_camera ${MY_STANDARD_LIBS})

rosbuild_add_executable(HistogramTest 
  src/Histogram.h
  src/HistogramTest.cpp)

rosbuild_add_executable(cameraModelTest 
  src/cameraModel.h
  src/cameraModel.cpp
  src/cameraModelTest.cpp)

rosbuild_add_executable(create_occlusion_by_background_mask 
  src/create_occlusion_by_background_mask.cpp
  src/ImageMask.h
  src/ImageMask.cpp)
target_link_libraries(create_occlusion_by_background_mask ${MY_STANDARD_LIBS})
