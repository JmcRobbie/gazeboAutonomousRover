cmake_minimum_required(VERSION 2.8.3)
project(autonomous_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  message_generation
  common
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EncoderData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include/autonomous_sim)

## Declare a C++ library
add_library(gazebo_ros_skid_steer_drive_custom src/gazebo_ros_skid_steer_drive_custom.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_ros_skid_steer_drive_custom ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(gazebo_ros_skid_steer_drive_custom autonomous_sim_gencpp)
