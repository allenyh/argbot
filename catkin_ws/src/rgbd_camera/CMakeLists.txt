cmake_minimum_required(VERSION 2.8.3)
project(rgbd_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  ddynamic_reconfigure
)

catkin_package(
)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_to_point src/depth_to_point.h src/depth_to_point.cpp)
target_link_libraries(depth_to_point ${catkin_LIBRARIES})

add_executable(pcl2depth src/pcl2depth.cpp)
target_link_libraries(pcl2depth ${catkin_LIBRARIES})