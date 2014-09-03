cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_state_publisher)

find_package(orocos_kdl REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  robot_state_publisher
  kdl_parser)

add_service_files(FILES
    AlterUrdf.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    message_runtime
    robot_state_publisher)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${orocos_kdl_INCLUDE_DIRS}
  include)
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/dynamic_robot_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})