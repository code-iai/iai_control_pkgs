cmake_minimum_required(VERSION 2.8.3)
project(loopback_controller_manager)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  urdf 
  robot_mechanism_controllers
  pr2_hardware_interface
  pr2_controller_manager
  pr2_mechanism_model
  robot_state_publisher
  hardware_interface)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Boost Eigen orocos_kdl)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
include_directories(
  SYSTEM
  ${EIGEN_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} loopback_controller_manager.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
  -L${orocos_kdl_LIBRARY_DIRS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
