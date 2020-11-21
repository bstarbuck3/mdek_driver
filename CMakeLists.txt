cmake_minimum_required(VERSION 2.8.3)
project(mdek_driver)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs geometry_msgs sensor_msgs rospy)

set(CMAKE_BUILD_TYPE Release)

add_message_files(
  FILES
  UWB.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs sensor_msgs
)
