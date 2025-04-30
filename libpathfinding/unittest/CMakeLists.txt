cmake_minimum_required(VERSION 3.4)

project(test_pathfinding)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

enable_testing()
find_package(GTest REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
add_executable(
  test_pathfinding
  test_pathfinding.cpp
)

target_link_libraries(
  test_pathfinding
  GTest::gtest_main
  libpathfinding
)

include(GoogleTest)
gtest_discover_tests(test_pathfinding)