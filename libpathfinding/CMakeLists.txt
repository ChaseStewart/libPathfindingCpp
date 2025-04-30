cmake_minimum_required(VERSION 3.4)
project(libpathfinding VERSION 1.0.1 DESCRIPTION "Library that provides pathfinding for sets of 4 or fewer quadcopters")

# add header files in the lib
include_directories(.) 


set(CMAKE_BUILD_TYPE Debug)
add_library(libpathfinding SHARED pathfinding.cpp)

set_target_properties(libpathfinding PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER libpathfinding/pathfinding.hpp
)