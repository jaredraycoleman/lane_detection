cmake_minimum_required(VERSION 2.8)
option(CUDA_GPU "USING GPU" OFF)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

if(CUDA_GPU)
    find_package(CUDA QUIET REQUIRED)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; 
        -O3 -lineinfo
        -gencode=arch=compute_62,code=sm_62
    )
    cuda_add_executable(detect detect.cu polifitgsl.cpp OPTIONS -std=c++11)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX-FLAGS} -std=c++11")
    add_executable(detect detect.cpp polifitgsl.cpp)
endif()

target_link_libraries(detect ${OpenCV_LIBS} ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY})
