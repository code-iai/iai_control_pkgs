cmake_minimum_required(VERSION 2.8.3)
project(iai_robot_mechanism_controllers)

find_package(catkin REQUIRED COMPONENTS 
  pluginlib
  roscpp
  std_msgs
  pr2_controller_interface
  pr2_mechanism_model
  pr2_mechanism_controllers
  pr2_mechanism_msgs
  nav_msgs
  filters
  control_toolbox
  realtime_tools
  pr2_controller_manager
  message_runtime)

find_package(Boost REQUIRED COMPONENTS thread)
# TODO(Georg): remove dependency on orocos_kdl once orocos_kdl
# version 1.1.102+ is released
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS TODO
#  DEPENDS Boost TODO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
 
link_directories(
  ${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/rosie_odometry.cpp
  src/multi_joint_velocity_controller.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES})

# TODO (install targets)
