cmake_minimum_required(VERSION 2.8.3)
project(iai_ft_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(wrench_stamper 
  src/iai_ft_utils/wrench_stamper.cpp)

target_link_libraries(wrench_stamper
  ${catkin_LIBRARIES})
