cmake_minimum_required(VERSION 3.10.3)

execute_process(COMMAND whoami OUTPUT_VARIABLE CURRENT_USER OUTPUT_STRIP_TRAILING_WHITESPACE)

project(map)

message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g")
# # set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# set(CMAKE_BUILD_TYPE "Debug")  
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

# add_definitions(-DPNUM=64)
# add_definitions(-DPUB_DEBUG=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  fake_moma
  rog_map
  message_generation
  random_map_generator
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)
# find_package(CASADI REQUIRED)

# find_package(Boost REQUIRED COMPONENTS serialization thread)

generate_messages(DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map
  CATKIN_DEPENDS roscpp roslib std_msgs visualization_msgs sensor_msgs message_runtime fake_moma random_map_generator
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(grid_map 
  src/grid_map.cpp
)

# add_dependencies(planner fake_moma_gencpp)
target_link_libraries(grid_map
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)
