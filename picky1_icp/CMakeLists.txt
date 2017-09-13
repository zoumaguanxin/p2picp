
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(picky1_icp)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

link_libraries(${PCL_LIBRARIES_DIRS})

add_executable(picky1_icp main.cpp)

target_link_libraries (picky1_icp ${PCL_LIBRARIES})
