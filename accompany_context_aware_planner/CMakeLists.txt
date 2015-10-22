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

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include)
		    
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#mysql
#include_directories(/usr/local/include/cppconn/)
#include_directories(/usr/local/include/)
link_directories(/usr/local/lib/) 


rosbuild_add_executable(accompany_context_aware_planner ros/src/ContextAwarePlanner.cpp
														ros/src/proxemics.cpp)

rosbuild_add_executable(getPotentialProxemicsLocations ros/src/getPotentialProxemicsLocations.cpp)

rosbuild_add_executable(getUserPose ros/src/getUserPose.cpp)

TARGET_LINK_LIBRARIES(accompany_context_aware_planner mysqlcppconn)
TARGET_LINK_LIBRARIES(getUserPose mysqlcppconn)
