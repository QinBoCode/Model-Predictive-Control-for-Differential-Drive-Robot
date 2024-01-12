cmake_minimum_required(VERSION 2.8.3)
project(eso_differential_drive)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_tracking
 CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ESO src/ESO.cpp)
target_link_libraries(ESO ${catkin_LIBRARIES})
add_dependencies(ESO trajectory_tracking_generate_messages_cpp)
