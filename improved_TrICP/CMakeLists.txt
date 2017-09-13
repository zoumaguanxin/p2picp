cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(improved_tricp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(improve_TrICP main.cpp)

target_link_libraries (improve_TrICP ${PCL_LIBRARIES})


