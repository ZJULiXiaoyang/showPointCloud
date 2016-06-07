cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mutilICP)

find_package(PCL  REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OPENCV_DEFINITIONS})

add_executable (mutilICP main.cpp pointCloud.cpp icp.h)
target_link_libraries (mutilICP ${PCL_LIBRARIES} ${OpenCV_LIBS})
