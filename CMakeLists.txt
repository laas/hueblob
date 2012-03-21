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
# OLD NODES
find_package(OpenCV REQUIRED)
message(STATUS OpenCV libs: ${OpenCV_LIBS})
rosbuild_add_library(hueblob
  src/libhueblob/object.cpp include/libhueblob/object.hh
  src/libhueblob/hueblob.cpp include/libhueblob/hueblob.hh)
target_link_libraries(hueblob ${OpenCV_LIBS})
# Nodes.
#  hueblob node.
rosbuild_add_executable(hueblob_node src/nodes/hueblob_node.cpp)
target_link_libraries(hueblob_node hueblob)
# OLDNODE

include(FindPkgConfig)
if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
   pkg_check_modules(GTK gtk+-2.0)
   if (GTK_FOUND)
      include_directories(${GTK_INCLUDE_DIRS})
      link_directories(${GTK_LIBRARY_DIRS})
      add_definitions(-DHAVE_GTK)
      # Hack around gdk-pixbuf move issues for now
      # http://ubuntuforums.org/showthread.php?t=1596930
      include_directories(/usr/include/gdk-pixbuf-2.0)
   endif(GTK_FOUND)
endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)


# Nodelet library
rosbuild_add_library(nodelet
  src/nodelets/monitor_nodelet.cpp
  src/nodelets/tracker_2d_nodelet.cpp
  src/nodelets/projector_nodelet.cpp
  src/nodelets/window_thread.cpp)

target_link_libraries(nodelet hueblob ${GTK_LIBRARIES})


rosbuild_add_executable(monitor src/nodes/monitor.cpp)
rosbuild_add_executable(tracker_2d src/nodes/tracker_2d.cpp)
rosbuild_add_executable(projector src/nodes/projector.cpp)

# fake_camera_synchronizer_node node.
rosbuild_add_executable(
  fake_camera_synchronizer_node
  src/nodes/fake_camera_synchronizer_node.cpp)

# Tests
rosbuild_add_gtest(object tests/object.cpp)
target_link_libraries(object hueblob ${OpenCV_LIBS} )

# Download test bag file if required.
rosbuild_download_test_data(
  http://www.laas.fr/~tmoulard/2010-11-29-18-07-39.bag
  data/2010-11-29-18-07-39.bag
  617eadf43514bdb7e9c36180506f8de0)
rosbuild_download_test_data(
  http://www.laas.fr/~tmoulard/2010-12-06-13-44-44.bag
  data/2010-12-06-13-44-44.bag
  e980b7522c3e230acbe1361ebf9c7f1e)
