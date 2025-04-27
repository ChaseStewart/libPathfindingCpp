/**
 * @file main.cpp
 * @author Chase E. Stewart
 * @date 26 April 2025
 * @brief Exercise libpathfinding types and functions
 */

#include <vector>
#include "pathfinding.hpp"

// Uncomment only 1 of {TEST_n...} so that render_results.py 
// can properly render a result
// obviously these are compile time checks, so you need to make clean; make
// after choosing a new test

//#define TEST_1
//#define TEST_2
//#define TEST_3
#define TEST_4

using namespace std;

/**
 * @brief Utilize types and methods from libpathfinding to execute and render a scenario
 * this is organized to provide a set of readymade tests
 */
int main(void)
{
    vector<pathfind_result> results;
    vector<obstacle> obstacles;
    vector<Point> agents;
    vector<Point> targets;

    Boundary bounds{Point(0.0, 0.0), Point(10.0, 10.0)};

#ifdef TEST_1
    // simple test with shortest path sanity checking
    // and a single simple convex hull case
    obstacles.push_back({Point(5.0, 5.0), 2.0});
    obstacles.push_back({Point(2.0, 2.0), 0.5});

    targets.push_back({Point(8.0, 9.0)});
    targets.push_back({Point(7.0, 9.0)});
    targets.push_back({Point(2.0, 1.0)});
    targets.push_back({Point(5.0, 2.0)});

    agents.push_back({Point(4.0, 7.0)});
    agents.push_back({Point(2.0, 9.0)});
    agents.push_back({Point(2.0, 3.0)});
    agents.push_back({Point(8.0, 2.0)});
#endif // TEST_1

#ifdef TEST_2
    // the "peapod" test, convex hull around
    // two obstacles in both directions
    obstacles.push_back({Point(3.0, 3.0), 1.0});
    obstacles.push_back({Point(6.5, 6.5), 1.0});

    targets.push_back({Point(9.5, 9.5)});
    targets.push_back({Point(9.8, 9.8)});

    agents.push_back({Point(1.2, 1.0)});
    agents.push_back({Point(0.1, 0.1)});
#endif // TEST_2

#ifdef TEST_3
    // force one cross then another
    // and undo them both sequentially
    obstacles.push_back({Point(5, 5), 1.0});

    targets.push_back({Point(6.0, 4.0)});
    targets.push_back({Point(0.1, 9.5)});
    targets.push_back({Point(9.8, 9.8)});
    targets.push_back({Point(9.9, 0.1)});

    agents.push_back({Point(1.2, 1.0)});
    agents.push_back({Point(9.7, 0.1)});
    agents.push_back({Point(0.2, 9.9)});
    agents.push_back({Point(4.0, 6.0)});
#endif // TEST_3

#if defined TEST_4
    // undo an X wrapped around a convex hull
    obstacles.push_back({Point(3, 7), 2.99});

    targets.push_back({Point(9.9, 9.9)});
    targets.push_back({Point(9.8, 9.7)});
    targets.push_back({Point(5.5, 9.5)});

    agents.push_back({Point(0.2, 1.0)});
    agents.push_back({Point(2.5, 0.5)});
    agents.push_back({Point(1.0, 4.0)});
#endif // TEST_4

    results = pathfind(bounds, agents, targets, obstacles);
    print_result(bounds, obstacles, results);
    return 0;
}
