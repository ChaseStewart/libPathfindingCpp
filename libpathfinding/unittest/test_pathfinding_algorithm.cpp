#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "libpathfinding/pathfinding.hpp"

using namespace std;

namespace {
    class PathfindingAlgorithmTest : public ::testing::Test {
  
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

    TEST_F(PathfindingAlgorithmTest, ValidateTest1) {

      /* Test exit if validate_input fails */
      cout << "TEST_1: invalid input" << endl;
      EXPECT_THROW(pathfind(bounds, agents, targets, obstacles), std::invalid_argument);

      /* Test no agents available */
      bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};

      obstacles.push_back({Point(5.0, 5.0), 2.0});
      obstacles.push_back({Point(2.0, 2.0), 0.5});
  
      targets.push_back({Point(8.0, 9.0)});
      targets.push_back({Point(7.0, 9.0)});
      targets.push_back({Point(2.0, 1.0)});
      targets.push_back({Point(5.0, 2.0)});

      cout << "TEST_2: no agents available" << endl;
      results = pathfind(bounds, agents, targets, obstacles);
      ASSERT_TRUE(results.empty());

      // TODO assert target/agent match-ups
    }
}
