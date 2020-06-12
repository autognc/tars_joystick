cmake_minimum_required(VERSION 2.8.3)
project(tars_joystick)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orbot_joy
  CATKIN_DEPENDS geometry_msgs joy roscpp
  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tars_joy_node src/joyDrivers.cpp src/controller.cpp)

target_link_libraries(tars_joy_node serial ${catkin_LIBRARIES})
