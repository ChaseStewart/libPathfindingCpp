#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "libpathfinding/pathfinding.hpp"

using namespace std;

namespace {
  class PathfindingTest : public ::testing::Test {

  protected:
    vector<pathfind_result> results;
    vector<obstacle> obstacles;
    vector<Point> agents;
    vector<Point> targets;
    Boundary bounds;
  
    virtual void SetUp() {
      results.clear();
      obstacles.clear();
      agents.clear();
      targets.clear();
      bounds.max_corner().set<0>(0.0);
      bounds.max_corner().set<1>(0.0);
      bounds.min_corner().set<0>(0.0);
      bounds.min_corner().set<1>(0.0);
    }

    virtual void TearDown() {
      results.clear();
      obstacles.clear();
      agents.clear();
      targets.clear();
      bounds.max_corner().set<0>(0.0);
      bounds.max_corner().set<1>(0.0);
      bounds.min_corner().set<0>(0.0);
      bounds.min_corner().set<1>(0.0);
    }
  };
  // Demonstrate some basic assertions.
  TEST_F(PathfindingTest, ValidateInputParams) {    

      /* Test blank input data (should fail on boundary box validation) */
      cout << "\tTEST_1: test empty data" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));
      bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};

      /* Test too many agents */
      agents.push_back(Point{0.0, 0.0});
      agents.push_back(Point{1.0, 0.0});
      agents.push_back(Point{2.0, 0.0});
      agents.push_back(Point{3.0, 0.0});
      agents.push_back(Point{4.0, 0.0});
      cout << "\tTEST_2: test too many agents" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));
  }


}
