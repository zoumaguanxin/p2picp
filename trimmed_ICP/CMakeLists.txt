cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(trimmed_ICP)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

link_libraries(${PCL_LIBRARIES_DIRS})

add_executable(trimmed_icp main.cpp)

target_link_libraries (trimmed_icp ${PCL_LIBRARIES})
