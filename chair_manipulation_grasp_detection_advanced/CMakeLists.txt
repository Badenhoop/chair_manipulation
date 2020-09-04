cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_detection_advanced)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
  pcl_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  cpd_wrapper
  nanoflann_wrapper
)

find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_INCLUDE_DIRS and LIBFCL_LIBRARIES
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
find_library(LIBFCL_LIBRARIES fcl HINTS ${LIBFCL_PC_LIBRARY_DIRS})

find_package(Qhull REQUIRED)
if (HAVE_QHULL_2011)
  add_definitions(-DCHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_HAVE_QHULL_2011)
endif ()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
)

set(
  LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
  ${QHULL_LIBRARIES}
  ${LIBFCL_LIBRARIES}
)

set(
  SOURCE_FILES
  include/chair_manipulation_grasp_detection_advanced/grasp_hypothesis.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_creator.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_element.h
  include/chair_manipulation_grasp_detection_advanced/grasp_sampler.h
  include/chair_manipulation_grasp_detection_advanced/model.h
  include/chair_manipulation_grasp_detection_advanced/utils.h
  include/chair_manipulation_grasp_detection_advanced/exception.h
  include/chair_manipulation_grasp_detection_advanced/wrench.h
  include/chair_manipulation_grasp_detection_advanced/gripper.h
  include/chair_manipulation_grasp_detection_advanced/transform.h
  include/chair_manipulation_grasp_detection_advanced/contact.h
  include/chair_manipulation_grasp_detection_advanced/multi_arm_grasp.h
  include/chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h
  include/chair_manipulation_grasp_detection_advanced/qhull_mutex.h
  include/chair_manipulation_grasp_detection_advanced/stopwatch.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_segmentation.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_registration.h
  include/chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_receiver.h
  include/chair_manipulation_grasp_detection_advanced/nonrigid_transform.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_matcher.h
  include/chair_manipulation_grasp_detection_advanced/point_cloud_segmentation_pipeline.h
  include/chair_manipulation_grasp_detection_advanced/create_grasp_database_pipeline.h
  include/chair_manipulation_grasp_detection_advanced/grasp_detection_pipeline.h
  include/chair_manipulation_grasp_detection_advanced/mesh_reconstruction_pipeline.h
  include/chair_manipulation_grasp_detection_advanced/statistics.h
  include/chair_manipulation_grasp_detection_advanced/ik_checker.h
  include/chair_manipulation_grasp_detection_advanced/robot.h
  src/model.cpp
  src/utils.cpp
  src/grasp_database.cpp
  src/grasp_database_creator.cpp
  src/grasp_sampler.cpp
  src/gripper.cpp
  src/wrench.cpp
  src/transform.cpp
  src/grasp_synthesizer.cpp
  src/qhull_mutex.cpp
  src/point_cloud_receiver.cpp
  src/point_cloud_registration.cpp
  src/point_cloud_preprocessor.cpp
  src/point_cloud_segmentation.cpp
  src/mesh_reconstruction.cpp
  src/point_cloud_matcher.cpp
  src/nonrigid_transform.cpp
  src/ik_checker.cpp
  src/robot.cpp
)

set(
  TEST_SOURCE_FILES
  test/test_common.h
  test/test_common.cpp
)

# Nodes

add_executable(create_grasp_database_node src/create_grasp_database_node.cpp src/create_grasp_database_pipeline.cpp ${SOURCE_FILES})
target_link_libraries(create_grasp_database_node ${LIBRARIES})

add_executable(grasp_detection_node src/grasp_detection_node.cpp src/grasp_detection_pipeline.cpp ${SOURCE_FILES})
target_link_libraries(grasp_detection_node ${LIBRARIES})

add_executable(point_cloud_segmentation_node src/point_cloud_segmentation_node.cpp src/point_cloud_segmentation_pipeline.cpp ${SOURCE_FILES})
target_link_libraries(point_cloud_segmentation_node ${LIBRARIES})

add_executable(mesh_reconstruction_node src/mesh_reconstruction_node.cpp src/mesh_reconstruction_pipeline.cpp ${SOURCE_FILES})
target_link_libraries(mesh_reconstruction_node ${LIBRARIES})

# Tests

add_executable(convert_mesh_test test/convert_mesh_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(convert_mesh_test ${LIBRARIES})

add_executable(grasp_sampler_test test/grasp_sampler_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(grasp_sampler_test ${LIBRARIES})

add_executable(gripper_test test/gripper_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(gripper_test ${LIBRARIES})

add_executable(wrench_test test/wrench_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(wrench_test ${LIBRARIES})

add_executable(load_database_test test/load_database_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(load_database_test ${LIBRARIES})

add_executable(mesh_reconstruction_test test/mesh_reconstruction_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(mesh_reconstruction_test ${LIBRARIES})

add_executable(point_cloud_preprocessor_test test/point_cloud_preprocessor_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(point_cloud_preprocessor_test ${LIBRARIES})

add_executable(registration_test test/registration_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(registration_test ${LIBRARIES})

add_executable(pcl_viewer test/pcl_viewer.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(pcl_viewer ${LIBRARIES})