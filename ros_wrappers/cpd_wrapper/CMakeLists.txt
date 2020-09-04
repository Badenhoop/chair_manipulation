cmake_minimum_required(VERSION 3.0.2)
project(cpd_wrapper)
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS
        cpd/include
    LIBRARIES cpd
)
option(WITH_TESTS "" OFF)
option(WITH_FGT "" ON)
add_subdirectory(cpd)