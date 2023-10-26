cmake_minimum_required(VERSION 3.0.2)
project(LaneKeepingSystem)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set (OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  xycar_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LaneKeepingSystem
#  CATKIN_DEPENDS OpenCV roscpp sensor_msgs std_msgs xycar_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(modules
  src/${PROJECT_NAME}/HoughTransformLaneDetector.cpp
  src/${PROJECT_NAME}/MovingAverageFilter.cpp
  src/${PROJECT_NAME}/PIDController.cpp
  src/${PROJECT_NAME}/LaneKeepingSystem.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  modules
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  
)