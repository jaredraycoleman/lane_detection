cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Threads)

set(BOOST_LIBS system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

add_executable(detect detect.cpp detector.cpp lane.cpp polifitgsl.cpp uartcommander.cpp)
target_link_libraries(detect ${OpenCV_LIBS} ${GSL_LIBRARY} ${Boost_LIBRARIES} ${GSL_CBLAS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} config++)
