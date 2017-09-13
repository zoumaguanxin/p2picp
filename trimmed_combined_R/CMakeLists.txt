cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(trimmed_combined_R)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

link_libraries(${PCL_LIBRARIES_DIRS})

add_executable(tcr_slam main.cpp)

target_link_libraries (tcr_slam ${PCL_LIBRARIES})
