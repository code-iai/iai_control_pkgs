cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDeb)

rosbuild_init(rosie_controller_manager)

#rosbuild_genmsg()
#rosbuild_gensrv()

#add_definitions(-fPIC)

rosbuild_add_boost_directories()

rosbuild_add_executable(loopback_controller_manager loopback_controller_manager.cpp)
rosbuild_link_boost(loopback_controller_manager thread)


