#include <vector>
#include <gtest/gtest.h>
#include "libpathfinding/pathfinding.hpp"

using namespace std;

// Demonstrate some basic assertions.
TEST(test_pathfinding, BasicAssertions) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

// Demonstrate some basic assertions.
TEST(test_pathfinding, ValidateInputParams) {
    
    vector<pathfind_result> results;
    vector<obstacle> obstacles;
    vector<Point> agents;
    vector<Point> targets;
    Boundary bounds;

    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};

    // Expect two strings not to be equal.
    EXPECT_STRNE("hello", "world");
    // Expect equality.
    EXPECT_EQ(7 * 6, 42);
}