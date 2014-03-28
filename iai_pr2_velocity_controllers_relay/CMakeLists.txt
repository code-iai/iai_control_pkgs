cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(iai_pr2_vel_ctrls_relay src/iai_pr2_velocity_controllers_relay/iai_pr2_velocity_controllers_relay.cpp)
