cmake_minimum_required(VERSION 2.8.3)
project(rosparam_expose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Set.srv
  Get.srv
  Load.srv
  Dump.srv
  Delete.srv
  List.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
