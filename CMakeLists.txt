cmake_minimum_required(VERSION 3.4)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/libpathfinding)
project(main)

set(CMAKE_CXX_STANDARD 14)

include_directories(libpathfinding)
add_subdirectory(libpathfinding)
add_subdirectory(libpathfinding/unittest)

add_executable(main main.cpp)

target_link_libraries(main PUBLIC libpathfinding)


