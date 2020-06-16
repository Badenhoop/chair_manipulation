cmake_minimum_required(VERSION 3.0.2)
project(chair_grasp_detection)

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

# ================================================
# In case we want to create a custom grasp message
# ================================================
#
#set(MSG_DEPS
#  geometry_msgs
#)
#
#set(MSG_FILES
#  Grasp.msg
#)
#
#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#generate_messages(DEPENDENCIES ${MSG_DEPS})
#catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(registration_detection_node src/registration_detection.cpp)
target_link_libraries(registration_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(depth_image_detection_node src/depth_image_detection.cpp)
target_link_libraries(depth_image_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})