cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(pcd_header_reader)

find_package(PCL 1.7 REQUIRED)

add_executable(pcd_struct_generator main.cpp)
target_link_libraries(pcd_struct_generator ${PCL_LIBRARIES})