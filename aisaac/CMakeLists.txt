cmake_minimum_required(VERSION 2.8.3)
project(aisaac)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  consai_msgs
  std_msgs
  message_generation
  aisaac_communication
)

add_service_files(
  FILES
  pid.srv
  Kick.srv
)

add_message_files(
  FILES
  Status.msg
  Ball_sub_params.msg
  Def_pos.msg
  Shutdown.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(command_senders_blue
    src/command_senders_blue.cpp
)
add_executable(command_senders_yellow
    src/command_senders_yellow.cpp
)
add_executable(command_sender
    src/command_sender.cpp
)
add_executable(sender_xbee_yellow
    src/sender_xbee_yellow.cpp
)
add_executable(sender_xbee_blue
    src/sender_xbee_blue.cpp
)
add_executable(sender_blue_JO2023
    src/sender_blue_JO2023.cpp
)
target_link_libraries(command_senders_blue
    ${catkin_LIBRARIES}
)
target_link_libraries(command_senders_yellow
    ${catkin_LIBRARIES}
)
target_link_libraries(command_sender
    ${catkin_LIBRARIES}
)
target_link_libraries(sender_xbee_yellow
    ${catkin_LIBRARIES}
)
target_link_libraries(sender_xbee_blue
    ${catkin_LIBRARIES}
)
target_link_libraries(sender_blue_JO2023
    ${catkin_LIBRARIES}
)

generate_messages(
DEPENDENCIES
 std_msgs)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)
