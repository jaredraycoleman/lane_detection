cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(CUDA QUIET REQUIRED)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

cuda_add_executable(detect detect.cu polifitgsl.cpp OPTIONS -std=c++11)
target_link_libraries(detect ${OpenCV_LIBS} ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY})
