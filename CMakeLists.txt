cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)


set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})


find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable( Video video.cpp )
target_link_libraries( Video ${OpenCV_LIBS} )
