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

/**
 * Succinct aliases for boost::geometry types
 * Using boost::geometry as a design choice
 * guarantees workable and convertible polygons and lines
 * but also guarantees all necessary algorithms associated with these
 * only Boost::geometry and STL are required for this library
 */
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>; ///< alias for boost.geometry's d2::point_xy<double>
using Boundary = bg::model::box<Point>; ///< alias for boost.geometry box<Point>
using Line = bg::model::linestring<Point>; ///< alias for boost.geometry linestring<Point>
using MultiLine = bg::model::multi_linestring<Point>; ///< alias for boost.geometry multi_linestring<Point>
using Polygon = bg::model::polygon<Point>; ///< alias for boost.geometry polygon<Point>
using MultiPolygon = bg::model::multi_polygon<Polygon>; ///< alias for boost.geometry multi_polygon<Point>

/**
 * convenience macro to call bg::dsv (delimeter-separated value)
 *  with preset parameters for Python serialization
 */
#define LP_PRINT_GEOM(x) bg::dsv(x, ",","(",")",",","[","]",",")

const int NUM_MAX_AGENTS = 4; ///< max number of agents allowed per the prompt

/**
 *  a circular "obstacle" with center and radius
 */
struct obstacle
{
   Point p; ///< position of circle center
   double radius; ///< radius of circle
};

/** information that an agent "bids" to a target
 * including its best path, the distance of that path
 * the agent (to erase() if this bid is accepted)
 */
struct agent_bids
{
   int agent_vect_idx; ///< agent position in vector (to erase if selected)
   Point agent; ///< position of the agent
   Line path; ///< the path this agent bids
   double distance; ///< the distance of this agent's path
};

/**
 * Struct that holds a selected {agent, target} pairing
 * and the path that the winning agent bid for this target
 */
struct pathfind_result
{
   int id; ///< unique ID for reference
   Point agent; ///< position of selected agent
   Point target; ///< position of target
   Line path; ///< path that accepted agent bid
};

/**
 * @brief Print the entire state to STDOUT, can be piped into a *.csv file and rendered by render_result.py
 * @param bounds Outer boundary Box
 * @param obstacles Vector of obstacles
 * @param results Vector of pathfind_results
 */
void print_result(Boundary &bounds, std::vector<obstacle>& obstacles, std::vector<pathfind_result>& results);

/**
 * @brief given boundaries, some agents, and some targets, select paths from agent to target
 * Throws std::runtime_error if a target is unreachable by any agent
 * Throws std::invalid_argument if there is a problem with the input parameters
 * @param bounds boundary Box struct
 * @param agents vector of all agents (represented by Point) to bid upon targets
 * @param targets vector of all targets (represented by Point) to be bid upon
 * @param obstacles vector of all circular obstacles
 * @return vector of pathfind_results, algorithm is complete
 */
std::vector<pathfind_result> pathfind(Boundary &bounds, std::vector<Point>& agents, std::vector<Point>& targets, std::vector<obstacle>& obstacles);

/**
 * @brief ensure that the input params are valid- pathfind() will call this and should not proceed if it fails
 * @param bounds boundary Box struct
 * @param agents provided vector of agents
 * @param targets provided vector of targets
 * @param obstacles provided vector of obstacles
 * @return true if all inputs are acceptable, else false
 */
bool is_valid_input_params(Boundary &bounds, std::vector<Point>& agents, std::vector<Point>& targets, std::vector<obstacle>& obstacles);

#endif  // __PATHFINDING_HPP
