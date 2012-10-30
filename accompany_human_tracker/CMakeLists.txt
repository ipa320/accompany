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

# check for libs
find_library(TIM_TRACKER TimTracker)
if(NOT TIM_TRACKER)
  message(FATAL_ERROR "Could not find 'TimTracker' library, please git clone 'git://basterwijn.nl/home/bterwijn/git/TimTracker.git' and make install")
endif(NOT TIM_TRACKER)

find_package( OpenCV REQUIRED )

include_directories ("${PROJECT_SOURCE_DIR}/src")
rosbuild_add_executable(human_tracker src/human_tracker.cpp src/MyTracker.cpp)
target_link_libraries(human_tracker ${TIM_TRACKER} ${OpenCV_LIBS})

rosbuild_add_executable(view_tracks src/view_tracks.cpp)
target_link_libraries(view_tracks ${OpenCV_LIBS} boost_program_options)

rosbuild_add_executable(tracker_test_data src/tracker_test_data.cpp)
