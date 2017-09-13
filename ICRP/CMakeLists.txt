
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ICRP)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(ICRP main.cpp)

target_link_libraries(ICRP ${PCL_LIBRARIES})