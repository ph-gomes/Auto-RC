cmake_minimum_required(VERSION 2.8.3)
project(road_line_detection)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  image_geometry
  image_proc
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS thread system date_time)

catkin_package(
   INCLUDE_DIRS lsd_1.6
   INCLUDE_DIRS controller
)

include_directories(${OpenCV_INCLUDE_DIRS})


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_library(
   lsd_1.6.0
   lsd_1.6/lsd.c
   lsd_1.6/lsd.h
   lsd_1.6/lsd_cmd.c
 )

 add_library(
   Road_Line_Processing
   src/Road_Line_Processing.hpp
   src/Road_Line_Processing.cpp
 )

 add_library(
   controller
   controller/controller.h
   controller/controller.c
 )

target_link_libraries(
   Road_Line_Processing
   lsd_1.6.0
   controller
   ${catkin_LIBRARIES}
 )

 add_executable(road_line_detection src/road_line_node.cpp)


 target_link_libraries(
   road_line_detection
   ${catkin_LIBRARIES}
   Road_Line_Processing
 )

