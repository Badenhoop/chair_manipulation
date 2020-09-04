cmake_minimum_required(VERSION 3.0.2)
project(nanoflann_wrapper)
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS
        nanoflann/include
)
option(BUILD_TESTS "" OFF)
add_subdirectory(nanoflann)