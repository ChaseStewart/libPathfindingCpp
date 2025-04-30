#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "libpathfinding/pathfinding.hpp"

using namespace std;

namespace {
  class PathfindingInputsTest : public ::testing::Test {

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

  /* Tests to validate boundary inputs */
  TEST_F(PathfindingInputsTest, ValidateInputBounds) {
    /* Test blank input data (should fail on boundary box validation) */
    cout << "\tTEST_1: test empty bounds" << endl;
    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    /* Test an expected input for bounds (should succeed) */
    bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};
    cout << "\tTEST_2: test reasonable bounds" << endl;
    EXPECT_TRUE(is_valid_input_params(bounds, agents, targets, obstacles));

    /**
     * Test a very tight input for bounds (should still succeed)
     * With more insight about the scale of the quadcopters and typical
     * workspace, we might refine what is acceptable or not for a boundary
     */
    bounds = {Point(0.0, 0.0), Point(0.01, 0.01)};
    cout << "\tTEST_3: test tight but acceptable bounds" << endl;
    EXPECT_TRUE(is_valid_input_params(bounds, agents, targets, obstacles));
  }

  /* Tests to validate agent inputs */
  TEST_F(PathfindingInputsTest, ValidateInputAgents) {

      bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};

      /* Test too many agents, expect false */
      agents.push_back(Point{0.1, 0.1});
      agents.push_back(Point{1.0, 1.0});
      agents.push_back(Point{2.0, 2.0});
      agents.push_back(Point{3.0, 3.0});
      agents.push_back(Point{4.0, 4.0});
      cout << "\tTEST_1: test too many agents" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

      /* Test Agent out of bounds, expect false */
      agents.clear();
      agents.push_back(Point{5.0, 5.0});
      agents.push_back(Point{0.1, 10.001});
      cout << "\tTEST_2: Agent out of bounds" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

      /* Test agent within obstacle, expect false */
      agents.clear();
      agents.push_back(Point{5.0, 5.0});
      obstacles.push_back({Point{5.0,4.0}, 1.1});
      cout << "\tTEST_3: Agent within obstacle" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

      /**
       * Test agent on boundary line, expect false
       * This test led to a boundary check switch from bg::covered_by
       * to bg::within for consistency on the boundary line
       */
      agents.clear();
      obstacles.clear();
      agents.push_back(Point{0.0, 0.0});
      cout << "\tTEST_4: Agent on boundary line" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

      /* Test agent on obstacle line, expect false */
      agents.clear();
      obstacles.clear();
      obstacles.push_back({Point{5.0,4.0}, 1});
      agents.push_back(Point{5.0, 5.0});
      cout << "\tTEST_5: Agent on obstacle line" << endl;
      EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

      /* Test  success with agents */
      agents.clear();
      obstacles.clear();
      obstacles.push_back({Point(5.0, 5.0), 1.0});

      agents.push_back({Point(1.2, 1.0)});
      agents.push_back({Point(9.7, 0.1)});
      agents.push_back({Point(0.2, 9.9)});
      agents.push_back({Point(4.0, 6.0)});
      cout << "\tTEST_5: Reasonable scenario" << endl;
      EXPECT_TRUE(is_valid_input_params(bounds, agents, targets, obstacles));
    }

  /* Tests to validate target inputs */
  TEST_F(PathfindingInputsTest, ValidateInputTargets) {

    bounds = {Point(0.0, 0.0), Point(10.0, 10.0)};

    /* Test more targets than agents, expect true */
    targets.push_back(Point{0.1, 0.1});
    targets.push_back(Point{1.0, 1.0});
    targets.push_back(Point{2.0, 2.0});
    targets.push_back(Point{3.0, 3.0});
    targets.push_back(Point{4.0, 4.0});
    cout << "\tTEST_1: test more targets than agents" << endl;
    EXPECT_TRUE(is_valid_input_params(bounds, agents, targets, obstacles));

    /* Test Terget out of bounds, expect false */
    targets.clear();
    targets.push_back(Point{5.0, 5.0});
    targets.push_back(Point{0.1, 10.001});
    cout << "\tTEST_2: Target out of bounds" << endl;
    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    /* Test target within obstacle, expect false */
    targets.clear();
    targets.push_back(Point{5.0, 5.0});
    obstacles.push_back({Point{5.0,4.0}, 1.1});
    cout << "\tTEST_3: Target within obstacle" << endl;
    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    /**
     * Test target on boundary line, expect false
     * This test led to a boundary check switch from bg::covered_by
     * to bg::within for consistency on the boundary line
     */
    targets.clear();
    obstacles.clear();
    targets.push_back(Point{0.0, 0.0});
    cout << "\tTEST_4: Target on boundary line" << endl;
    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    /* Test target on obstacle line, expect false */
    targets.clear();
    obstacles.clear();
    obstacles.push_back({Point{5.0,4.0}, 1});
    targets.push_back(Point{5.0, 5.0});
    cout << "\tTEST_5: Target on obstacle line" << endl;
    EXPECT_FALSE(is_valid_input_params(bounds, agents, targets, obstacles));

    /* Test success with targets */
    targets.clear();
    obstacles.clear();
    obstacles.push_back({Point(5.0, 5.0), 1.0});

    targets.push_back({Point(1.2, 1.0)});
    targets.push_back({Point(9.7, 0.1)});
    targets.push_back({Point(0.2, 9.9)});
    targets.push_back({Point(4.0, 6.0)});
    cout << "\tTEST_5: Reasonable scenario" << endl;
    EXPECT_TRUE(is_valid_input_params(bounds, agents, targets, obstacles));
  }

}
