cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(karto)

set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Build Karto
#execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.kartoSDK
#                RESULT_VARIABLE _make_failed)
#if(_make_failed)
#  message(FATAL_ERROR "Build of Karto failed")
#endif(_make_failed)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# Build the ROS wrapper
include_directories(${PROJECT_SOURCE_DIR}/KartoSDK-2.1.4.6711-Linux-64/include)
link_directories(${PROJECT_SOURCE_DIR}/KartoSDK-2.1.4.6711-Linux-64/lib)

rosbuild_add_executable(slam_karto src/slam_karto.cpp)
rosbuild_link_boost(slam_karto signals)
target_link_libraries(slam_karto Karto_64)

rosbuild_add_executable(slam_ctl src/slam_ctl.cpp)


# Dynamic reconfiguration to enable/disable incorporation of laser scans and
# map updates at runtime.
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
