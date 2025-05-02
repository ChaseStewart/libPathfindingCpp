# libPathfindingCpp
[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](https://chasestewart.github.io/libPathfindingCpp/)

A C++ path planner providing an algorithm to assign agents to targets.

See [Test Results](#test-results) to get a sense of what this does.

## Author
Chase E. Stewart

## Unittests
**In order to ensure maximum compatibility and ease of demonstration, this `main` branch + README instruction set uses the Makefile system with gcc-c++ compiler and does not include unittests.**

The reason this Makefile-style branch does not include unittests is that it would need to include an unvetted GHA, a git submodule (not terrible but sort of clunky), or worst case a static copy of some amount of GoogleTest source
in order for the unittests to run. I decided it would be preferable to convert the project to CMake to both show off that and also to use the elegant `FetchContent()` module. However, I did encounter difficulty with WSL/Ubuntu version when trying to run the CMake unittests on another computer, so rather than risk incompatibility for another user, I am offering both the more supported Makefile version and the CMake version with unittests developed. 

Again, unittests have been written within the [CMake Branch](https://github.com/ChaseStewart/libPathfindingCpp/tree/convert_Makefile_to_CMakeLists.txt)- please check out that branch
and use its README instructions to compile and run unittests for libpathfinding. The unittest executable `test\_pathfinding` covers `is_valid_input_params()` and some amount of `pathfind()` from the library-
it is not extensive but is intended to be enough to demonstrate skill and design in setting up and using Google Test, CMakeLists.txt, and general unittest design principles. 

## Structure
library libpathfinding/ is a shared library that provides a wrapper over [boost::geometry](https://www.boost.org/doc/libs/1_85_0/libs/geometry/doc/html/index.html)
so as to provide a desired algorithm for multi-quadcopter pathfinding.

Directories and files of note in this repository:
* _documentation/:_ a directory holding the Doxyfile for generating Doxygen documentation
* _extra/:_ folder with DroneStatus.msg
* _libpathfinding/:_ a directory holding the shared library for the path algorithm
* _results/:_ a folder with .png images of the library working on main.cpp's tests
* _main.cpp:_ an example function pre-loaded with some tests for `libpathfinding/` described in [Test Results](#test-results)
* _render_results.py:_ a Python3 script that renders outputs of libpathfinding's `print\_result()` via matplotlib. **It requires input filename be `results.csv`**

## Setup
### Pre-Installation
From an absolutely unmodified version of WSL Ubuntu-24.04

* First, clone this repository
```shell
git clone git@github.com:ChaseStewart/libPathfindingCpp.git
```
* Then, install the boost library
```shell
sudo apt-get install libboost-all-dev
```
* Install python and its needed libraries, if you haven't
```shell
sudo apt-get install python3
sudo apt-get install python3-matplotlib
sudo apt-get install python3-numpy
```
Now you will be ready to compile and run the program

### Installation and Running
Follow these steps to exercise the library example

* From `libPathfindingCpp/`, call `make`
* [Optional] Run `./main` and observe outputs
* Run `./main > results.csv` **NOTE: you must name output file `results.csv` for simplicity**
* run `./render_results.py` **NOTE: your terminal must be capable of popping up windows- my WSL from Windows 11 can do this**
  + in case you have trouble with this, I have provided captures of the test results. Results should be deterministic.

## Design
### Goals
As far as the high level design of this project- I wanted to accomplish a few things:
* Try to use idiomatic, maintainable, and extensible methods as much as possible
* Separate aspects of algorithm from usage from the start and provide a sensible interface
* Show my own reasoning- for example don't use vibe coding to jam the prompt into a format for A* search and call it a day
* Try to show some algorithmic thinking, including anticipating and resolving corner cases

### Philosophy
So then, I thought about this problem and broke it down like this:
* There will only ever be 4 quadcopters, so algorithmic complexity is not quite punishing here
   + Where there are trade-offs, we want to optimize for correctness
   + Double and triple checking against crossed paths is a decision I am willing to defend
* I am approaching this geometrically rather than turning some grid size of the 2D space into nodes for a search
   + this is an informed decision based on the constraints  of what can happen in this task 
* We can leverage boost::geometry's concepts of linestrings, polygons, and algorithms to manage complexity
   + I did not fuss about the computational complexity of boost::geometry algorithms- again there are only 4 quadcopters
* All circles are convex shapes, so **we will use the [convex_hull algorithm](https://en.wikipedia.org/wiki/Convex_hull) to tightly pathfind around one or many obstacles**
   + First turn the line into a thin stroked path, then do a boolean OR of the path-line with all obstacles in the way, then do a convex hull of that compound shape
   + Try one side of that compound shape, then the other if the first goes out of bounds
   + union (boolean OR) and convex\_hull are both implemented in boost::geometry and are simple to use with my types
   + Additionally, intersects and 
   + So then, we should either have a clockwise or counterclockwise path around any legal combination of non-touching obstacles
   + touching obstacles would create a non-circular keepout area, and thus are considered illegal
* To handle multiple paths wrapping around an obstacle, there is a sort of "backoff" of extra buffer around obstacles for subsequent agents
   + if the crossed paths cannot be resolved, this backoff will eventually lead to the algorithm failing with an exception
   + This should only (rightly) trigger in devilishly complex scenarios where I can't even talk through a fix with the picture in front of me

### High Level Algorithm
The high level algorithm is as follows:
1. validate all inputs, exit with error if any invalid case such as obstacle fully covers boundary or more than 4 agents
2. Iterate over each target in order added, each available agent bids a path and distance
3. For their bids, agents prefer straight line paths if acceptable, or else generate curved line paths (described below) as necessary
4. After all available agents bid, the target accepts the bid with shortest distance, selected agent is removed from further consideration
5. If we run out of agents before targets, we skip the rest of targets, and no further paths are generated
6. After all targets have accepted bids, or after agents run out, if there are two or more accepted bids, we check for intersections
7. For every combination [i,j]; i != j && i < accepted\_bids  && j < accepted\_bids in reverse order, check for intersection between paths
8. If there is an intersection, i and j swap their agents, and their paths are recalculated without re-bidding
9. repeat steps 6 - 8 until we get through combinations without an intersection- if this goes indefinitely, eventually algorithm will raise exception
10. return list of paths

### Challenges
There were definitely a few challenges here:
* First approach was going to be to take a straight path, then an intersection of a buffer around the circle so like a beeline, then half-circle, then beeline again. However the boost::geometry tools would have made this quite challenging, as I would need to perhaps pull in boost::polygon or get deep into polygon outer-rings and directions and manually sew polygons
* I was attempting to make the clockwise/counterclockwise traversal of the convex hull way too hard at first for reasons that are now hard to explain
* There was some challenge working with polygons and multi-polygons in the convex hull algorithm, turned out boost::geometry was very intuitive and let me index multi-polygons like vectors
* Some time was wasted with the uncrossing for-loop counting down from final\_results.size()-1 -> 0 as I made the var an unsigned size\_t and was counting while > 0, just one of those silly mistakes. GDB helped me see what I was doing.
* Before I set some constant values out to the top of pathfinding.cpp, I had some trouble tuning those values for TEST\_4 which I made extremely hard on myself. This was probably good in the long term, but I spent a lot of time getting that working

## Test Results
### Test\_1 Results
Input data
```c++
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
```
![Screenshot](/results/Test1_output.png)
### Test\_2 Results
Input data
```c++
    // the "peapod" test, convex hull around
    // two obstacles in both directions
    obstacles.push_back({Point(3.0, 3.0), 1.0});
    obstacles.push_back({Point(6.5, 6.5), 1.0});

    targets.push_back({Point(9.5, 9.5)});
    targets.push_back({Point(9.8, 9.8)});

    agents.push_back({Point(1.2, 1.0)});
    agents.push_back({Point(0.1, 0.1)});
```
![Screenshot](/results/Test2_output.png)
### Test\_3 Results
Input data
```c++
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
```
![Screenshot](/results/Test3_output.png)
### Test\_4 Results
Input data
```c++
    // undo an X wrapped around a convex hull
    obstacles.push_back({Point(3, 7), 2.99});

    targets.push_back({Point(9.9, 9.9)});
    targets.push_back({Point(9.8, 9.7)});
    targets.push_back({Point(5.5, 9.5)});

    agents.push_back({Point(0.2, 1.0)});
    agents.push_back({Point(2.5, 0.5)});
    agents.push_back({Point(1.0, 4.0)});
```
![Screenshot](/results/Test4_output.png)
