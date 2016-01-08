cmake_minimum_required(VERSION 2.8.3)
project(iai_naive_kinematics_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iai_naive_kinematics_sim
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(iai_naive_kinematics_sim
#   src/${PROJECT_NAME}/iai_naive_kinematics_sim.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(iai_naive_kinematics_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(iai_naive_kinematics_sim_node src/iai_naive_kinematics_sim_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(iai_naive_kinematics_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(iai_naive_kinematics_sim_node
#   ${catkin_LIBRARIES}
# )

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
