cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_planning)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    shape_msgs
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    chair_manipulation_msgs
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

set(SOURCE_FILES
    include/chair_manipulation_grasp_planning/grasp_planner.h
    include/chair_manipulation_grasp_planning/utils.h
    src/utils.cpp
    src/grasp_planner.cpp
)

add_executable(move_node src/move_node.cpp ${SOURCE_FILES})
target_link_libraries(move_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(grasp_planner_node src/grasp_planner_node.cpp ${SOURCE_FILES})
target_link_libraries(grasp_planner_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(gripper_command_action_server_node src/gripper_command_action_server_node.cpp ${SOURCE_FILES})
target_link_libraries(gripper_command_action_server_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
