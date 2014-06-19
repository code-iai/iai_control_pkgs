cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(joint_state_restamper 
    src/iai_rosbag_utils/joint_state_restamper.cpp)
rosbuild_add_executable(pr2_fingertip_restamper 
    src/iai_rosbag_utils/pr2_fingertip_restamper.cpp)
rosbuild_add_executable(human_restamper 
    src/iai_rosbag_utils/human_restamper.cpp)
rosbuild_add_executable(equipment_restamper 
    src/iai_rosbag_utils/equipment_restamper.cpp)
rosbuild_add_executable(tf_restamper 	
	src/iai_rosbag_utils/tf_restamper.cpp)