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

SET( OPENCV_INCLUDE_DIR /usr/local/include)
SET( OPENCV_LIB_DIR /usr/local/lib)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
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

rosbuild_add_executable(BuildBackgroundModel src/BuildBackgroundModel.cpp)

rosbuild_add_executable(CreatePrior src/CreatePrior.cpp)

rosbuild_add_executable(CameraLocalisation src/CameraLocalisation.cpp)
target_link_libraries(CameraLocalisation boost_program_options)
