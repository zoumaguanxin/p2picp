project(improvedTrICP)
cmake_minimum_required(VERSION 2.6)
find_package(Qt4 REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

set(improvedTrICP_SRCS improvedTrICP.cpp main.cpp)
qt4_automoc(${improvedTrICP_SRCS})
add_executable(improvedTrICP ${improvedTrICP_SRCS})
target_link_libraries(improvedTrICP ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
install(TARGETS improvedTrICP RUNTIME DESTINATION bin)
