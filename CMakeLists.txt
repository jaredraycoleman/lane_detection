cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

add_executable(detect detect.cpp polifitgsl.cpp)
target_link_libraries(detect ${OpenCV_LIBS} ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY})
