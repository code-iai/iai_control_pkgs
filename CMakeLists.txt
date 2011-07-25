cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()
rosbuild_add_library(ias_mechanism_controllers
    src/multi_joint_velocity_controller.cpp
)


rosbuild_remove_compile_flags(ias_mechanism_controllers -W)
target_link_libraries(ias_mechanism_controllers ltdl)
