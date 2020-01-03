cmake_minimum_required(VERSION 2.8.3)
project(panda_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  joint_state_controller
  joint_state_publisher
  joint_trajectory_controller
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  xacro
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(lib_NE src/NE_matrix.cpp)

# add_executable(robot_control_node  src/robot_control_node.cpp)
# add_executable(box_publisher_node  src/box_publisher.cpp)
add_executable(impedance_node
  src/impedance.cpp
  src/NE_matrix.cpp)
target_link_libraries(impedance_node ${catkin_LIBRARIES}
  boost_filesystem ${JSONCPP_LIBRARIES} lib_NE)
