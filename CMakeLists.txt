cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#gensrv()

rosbuild_add_library(ias_mechanism_controllers src/rosie_odometry.cpp
                                               src/multi_joint_velocity_controller.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(ias_mechanism_controllers thread)


