cmake_minimum_required(VERSION 3.10.3)

execute_process(COMMAND whoami OUTPUT_VARIABLE CURRENT_USER OUTPUT_STRIP_TRAILING_WHITESPACE)

project(planner)

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

add_definitions(-DPNUM=64)
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
  pybind11_catkin
  fake_moma
  rog_map
  message_generation
  random_map_generator
  map
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)
find_package(CASADI REQUIRED)
find_package(TBB REQUIRED)
find_package(Python3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization thread)

add_message_files(FILES
  PlotState.msg
  MeshPart.msg
  MeshState.msg
  MeshTraj.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner
  CATKIN_DEPENDS roscpp roslib std_msgs visualization_msgs sensor_msgs message_runtime pybind11_catkin fake_moma random_map_generator map
)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
)

add_library(planner 
  src/ompc.cpp
  src/graph_search.cpp
  src/birrts.cpp
  src/utils/raycast.cpp
  src/topo_prm.cpp
  src/mcrrts.cpp
  src/ompls.cpp
  src/moma_traj_opt.cpp
  src/planner.cpp
)

add_dependencies(planner fake_moma_gencpp)
target_link_libraries(planner
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  pybind11::module
  OsqpEigen::OsqpEigen
  ${pybind11_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable (planner_node src/planner_node.cpp )
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  planner
  ${OMPL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CASADI_LIBRARIES}
  ${Boost_LIBRARIES}
)

# pybind_add_module(data 
#   MODULE
#   src/utils/data.cpp)

# target_link_libraries(
#   data
#   PRIVATE pybind11::module
#   ${Boost_LIBRARIES}
# )

# pybind_add_module(shared_data
#   MODULE
#   src/utils/shared_data.cpp
# )

# target_link_libraries(
#   shared_data
#   PRIVATE pybind11::module
#   ${Boost_LIBRARIES}
#   rt
# )
