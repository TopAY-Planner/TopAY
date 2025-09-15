cmake_minimum_required(VERSION 2.8.3)
project(fake_moma)

ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# set(CMAKE_BUILD_TYPE "Debug")  
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  tf
  roslib
  pcl_conversions
  visualization_msgs
  ackermann_msgs
  pybind11_catkin
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(FILES
  MomaCmd.msg
  MomaState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  nav_msgs
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fake_moma
  CATKIN_DEPENDS roscpp std_msgs tf visualization_msgs message_runtime ackermann_msgs pybind11_catkin
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(fake_moma src/moma_sim.cpp)
add_dependencies(fake_moma fake_moma_gencpp)

add_executable (fake_moma_node src/moma_sim.cpp)
add_dependencies(fake_moma_node fake_moma_gencpp)
target_link_libraries(fake_moma_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (moma_vis_node src/moma_vis.cpp)
add_dependencies(moma_vis_node fake_moma_gencpp)
target_link_libraries(moma_vis_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

pybind_add_module(moma_param 
  MODULE
  src/moma_param.cpp)

target_link_libraries(
  moma_param
  PRIVATE pybind11::module
  ${pybind11_LIBRARIES}
)