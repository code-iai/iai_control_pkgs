cmake_minimum_required(VERSION 2.8.3)
project(iai_visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  saphari_msgs
  visualization_msgs)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    saphari_msgs 
    visualization_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(human_visualization 
  src/iai_visualization/human_visualization.cpp)
target_link_libraries(human_visualization
  ${catkin_LIBRARIES})
add_dependencies(human_visualization ${catkin_EXPORTED_TARGETS})


add_executable(equipment_visualization 	
  src/iai_visualization/equipment_visualization.cpp)
target_link_libraries(equipment_visualization 	
  ${catkin_LIBRARIES})
add_dependencies(equipment_visualization ${catkin_EXPORTED_TARGETS})

