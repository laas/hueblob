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

rosbuild_genmsg()
rosbuild_gensrv()

# Library.
rosbuild_add_library(hueblob
  src/libhueblob/object.cpp include/libhueblob/object.hh
  src/libhueblob/hueblob.cpp include/libhueblob/hueblob.hh)

# Nodes.
#  hueblob node.
rosbuild_add_executable(hueblob_node src/nodes/hueblob_node.cpp)
target_link_libraries(hueblob_node hueblob)

# fake_camera_synchronizer_node node.
rosbuild_add_executable(
  fake_camera_synchronizer_node
  src/nodes/fake_camera_synchronizer_node.cpp)

# Tests
rosbuild_add_gtest(object tests/object.cpp)
target_link_libraries(object hueblob)

# Download test bag file if required.
rosbuild_download_test_data(
  http://www.laas.fr/~tmoulard/2010-11-29-18-07-39.bag
  data/2010-11-29-18-07-39.bag
  617eadf43514bdb7e9c36180506f8de0)
