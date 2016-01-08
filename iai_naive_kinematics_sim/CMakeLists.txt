cmake_minimum_required(VERSION 2.8.3)
project(iai_naive_kinematics_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  sensor_msgs
  std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp urdf sensor_msgs std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS iai_naive_kinematics_sim iai_naive_kinematics_sim_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

set(TEST_SRCS
  test/${PROJECT_NAME}/main.cpp
  test/${PROJECT_NAME}/simulator.cpp
  test/${PROJECT_NAME}/system.cpp
  test/${PROJECT_NAME}/watchdog.cpp)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
