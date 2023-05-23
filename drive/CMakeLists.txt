cmake_minimum_required(VERSION 3.0.2)
project(drive LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  xycar_msgs
  #OpenCV REQUIRED HINTS ${CMAKE_SOURCE_DIR}
)

find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drive
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs xycar_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(driver src/linedetect.cpp)

target_link_libraries(driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)