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

rosbuild_find_ros_package(learning_machine)
include_directories(${learning_machine_PACKAGE_PATH}/include)
#include_directories(${learning_machine_PACKAGE_PATH}/libsvm-3.16)

rosbuild_find_ros_package(hiro_common)
include_directories(${hiro_common_PACKAGE_PATH}/include)

# For matlab engine
include_directories(/usr/local/matlab/R2009b/extern/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_library(${PROJECT_NAME} ${learning_machine_PACKAGE_PATH}/libsvm-3.16/svm.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# set the path to the library folder
link_directories(/usr/local/matlab/R2009b/bin/glnxa64)

rosbuild_add_executable(general_mgr src/general_manager.cpp)
target_link_libraries(general_mgr boost_filesystem boost_system lwpr log4cxx eng mat mx ut mwfl boost_date_time icudata icuuc icui18n icuio boost_filesystem boost_regex hdf5_hl hdf5 boost_thread-gcc42-mt-1_36 boost_signals-gcc42-mt-1_36 boost_system-gcc42-mt-1_36) 
