cmake_minimum_required(VERSION 2.8.3)
project(iai_rosbag_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pr2_msgs
  saphari_msgs
  tf2_msgs
  visualization_msgs)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    pr2_msgs
    saphari_msgs
    tf2_msgs
    visualization_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(joint_state_restamper 
  src/iai_rosbag_utils/joint_state_restamper.cpp)
target_link_libraries(joint_state_restamper 
  ${catkin_LIBRARIES})
add_dependencies(joint_state_restamper
  ${catkin_EXPORTED_TARGETS})

add_executable(pr2_fingertip_restamper 
  src/iai_rosbag_utils/pr2_fingertip_restamper.cpp)
target_link_libraries(pr2_fingertip_restamper 
  ${catkin_LIBRARIES})
add_dependencies(pr2_fingertip_restamper
  ${catkin_EXPORTED_TARGETS})

add_executable(human_restamper 
  src/iai_rosbag_utils/human_restamper.cpp)
target_link_libraries(human_restamper 
  ${catkin_LIBRARIES})
add_dependencies(human_restamper
  ${catkin_EXPORTED_TARGETS})

add_executable(equipment_restamper 
  src/iai_rosbag_utils/equipment_restamper.cpp)
target_link_libraries(equipment_restamper 
  ${catkin_LIBRARIES})
add_dependencies(equipment_restamper
  ${catkin_EXPORTED_TARGETS})

add_executable(tf_restamper
  src/iai_rosbag_utils/tf_restamper.cpp)
target_link_libraries(tf_restamper	
  ${catkin_LIBRARIES})
add_dependencies(tf_restamper
  ${catkin_EXPORTED_TARGETS})

add_executable(tf_repeater
  src/iai_rosbag_utils/tf_repeater.cpp)
target_link_libraries(tf_repeater
  ${catkin_LIBRARIES})
add_dependencies(tf_repeater
  ${catkin_EXPORTED_TARGETS})
