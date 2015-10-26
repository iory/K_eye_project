cmake_minimum_required(VERSION 2.8.3)
project(eye_project)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  rosserial_arduino
  rosserial_client
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
