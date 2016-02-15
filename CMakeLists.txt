cmake_minimum_required(VERSION 2.8.3)
project(eye_see)

include(dlib/cmake)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_generation
  message_runtime
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  sensor_msgs
  std_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

MACRO(add_example name)
   ADD_EXECUTABLE(${name} src/${name}.cpp)
   TARGET_LINK_LIBRARIES(${name} dlib )
ENDMACRO()

add_example(face_detection)
TARGET_LINK_LIBRARIES(face_detection dlib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(face_position src/face_position_publisher.cpp)
target_link_libraries(face_position ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
