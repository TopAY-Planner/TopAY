cmake_minimum_required(VERSION 2.8.3)
project(random_map_generator)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  roslib
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(Boost 1.71 REQUIRED COMPONENTS system filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES random_map_generator
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(random_map_generator src/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (random_map src/random_map_node.cpp )
target_link_libraries(random_map
  ${catkin_LIBRARIES}
  random_map_generator
  ${PCL_LIBRARIES}
)
