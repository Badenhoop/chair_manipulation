cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_detection_simple)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf_conversions
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(depth_image_detection_node src/depth_image_detection_node.cpp)
target_link_libraries(depth_image_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})