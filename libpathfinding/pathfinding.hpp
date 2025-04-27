/**
 * @file pathfinding.hpp
 * @author Chase E. Stewart
 * @date 4/24/2025
 * @brief Library header file for libpathfinding
 */
#ifndef __PATHFINDING_HPP_
#define __PATHFINDING_HPP_

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

// Succinct aliases for boost::geometry types
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Boundary = bg::model::box<Point>;
using Line = bg::model::linestring<Point>;
using MultiLine = bg::model::multi_linestring<Point>;
using Polygon = bg::model::polygon<Point>;
using MultiPolygon = bg::model::multi_polygon<Polygon>;

// convenience macro to call DSV (delimeter-separated value)
// with values useful for Python serialization
#define LP_PRINT_GEOM(x) bg::dsv(x, ",","(",")",",","[","]",",")

const int NUM_MAX_AGENTS = 4;

// a circular "obstacle"
struct obstacle
{
   Point p; // centerpoint of circle
   double radius; // radius of circle
};

struct agent_bids
{
   int agent_vect_idx;  // 
   Point agent;
   Line path;
   double distance;
};

/**
 * effectively, an {agent, target} pairing
 * that can be converted into a straight line
 */
struct pathfind_result
{
   int id; // unique ID for reference
   Point agent; // starting point
   Point target; // ending point
   Line path; // linestring path
};

/**
 * @brief Print the entire state to STDOUT, can be piped into a *.csv file and rendered
 * 
 * @param bounds Outer boundary Box
 * @param obstacles Vector of 
 * @param results Vector of {agent, target} pairings and unique ID
 */
void print_result(Boundary &bounds, std::vector<obstacle>& obstacles, std::vector<pathfind_result>& results);

/**
 * @brief given boundaries, some agents, and some targets, select paths from agent to target
 * Throws std::invalid_argument if a target is unreachable by any agent
 */
std::vector<pathfind_result> pathfind(Boundary &bounds, std::vector<Point>& agents, std::vector<Point>& targets, std::vector<obstacle>& obstacles);

/**
 * @brief ensure that the input params are valid
 * @param bounds boundary Box struct
 * @param agents provided vector of agents
 * @param targets provided vector of targets
 * @param obstacles provided vector of obstacles
 * @return true if all inputs are acceptable, else false
 */
bool is_valid_input_params(Boundary &bounds, std::vector<Point>& agents, std::vector<Point>& targets, std::vector<obstacle>& obstacles);

#endif  // __PATHFINDING_HPP