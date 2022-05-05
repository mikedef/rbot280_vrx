cmake_minimum_required(VERSION 3.0.2)
project(rbot_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Classification.msg
  ClassificationArray.msg
#   Message1.msg
#   Message2.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rbot_msgs
  CATKIN_DEPENDS 
  sensor_msgs
  message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


