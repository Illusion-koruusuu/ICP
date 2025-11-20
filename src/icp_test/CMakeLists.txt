cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(icp_test)

find_package(PCL 1.5 REQUIRED)


add_executable (icp_test icp.cpp)
target_link_libraries (icp_test ${PCL_LIBRARIES})