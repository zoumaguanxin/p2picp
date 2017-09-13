cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(slam6)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

link_libraries(${PCL_LIBRARIES_DIRS})

add_executable(vcpslam main.cpp)

target_link_libraries (vcpslam ${PCL_LIBRARIES})
