cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)

option(JETSON_TX2 "Build for the Jetson TX2" OFF)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Threads)

find_package(Boost 1.58.0 REQUIRED COMPONENTS system log)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

if(JETSON_TX2)
    find_package(CUDA QUIET REQUIRED)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};
        -O3 -lineinfo
        -gencode=arch=compute_62,code=sm_62
    )
    cuda_add_executable(detect detect.cpp detector.cu lane.cpp polifitgsl.cpp uartcommander.cpp OPTIONS -std=c++11)
else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")
    add_executable(detect helpers.cpp detect.cpp detector.cpp lane.cpp polifitgsl.cpp uartcommander.cpp pid.cpp)
endif()

target_link_libraries(detect ${OpenCV_LIBS} ${GSL_LIBRARY} ${Boost_LIBRARIES} ${GSL_CBLAS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} config++)
