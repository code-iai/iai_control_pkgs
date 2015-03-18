cmake_minimum_required(VERSION 2.8.3)
project(iai_ros_controllers)

find_package(catkin REQUIRED roscpp controller_interface)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iai_ros_controllers
  CATKIN_DEPENDS roscpp controller_interface
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/MultiJointVelocityImpedanceController.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
