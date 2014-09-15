cmake_minimum_required(VERSION 2.8.3)
project(urdf_management)

find_package(catkin REQUIRED COMPONENTS
    message_generation)

add_service_files(FILES
    AlterUrdf.srv)

generate_messages(DEPENDENCIES)

catkin_package(
    DEPENDS message_runtime)


