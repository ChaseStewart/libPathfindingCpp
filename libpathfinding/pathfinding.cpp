/**
 * @file pathfinding.cpp
 * @author Chase E. Stewart
 * @date 4/24/2025
 * @brief Library source for libpathfinding
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/dsv/write.hpp>

#include "pathfinding.hpp"

using namespace std;
namespace bg = boost::geometry;


// tunables
const int points_per_circle = 16; ///< configurable number of points around circles
const double line_buffer_distance = 0.1; ///< configurable relatively small "stroke-width" to turn lines to polygons
const float min_keepout_buffer = 0.05; ///< scalar for get_obstacle_buffer_size()

/**
 *  static variable, ensures n-many wraps around obstacles don't take same path
 *  each subsequent call to get_obstacle_avoid_path will have additional keepout
 */
static int buffer_offset = 1; ///< incrementing value to increase subsequent keepout around obstacles

/* Resolving paths */
static void swap_agents(vector<pathfind_result> &pr, int idx_1, int idx_2);
static Line get_obstacle_avoid_path(Line straight_path, vector<obstacle> &obstacles, bool is_clockwise);
static Line find_convex_hull_subset(Point agent, Point target, Line convex_hull, bool is_clockwise);
static Line calculate_path(Boundary &bounds, Point agent, Point target, vector<obstacle> &obstacles);

/* boundary checking */
static vector<obstacle> get_intersecting_obstacles(Line straight_path, vector<obstacle> obstacles);
static bool is_path_crossing(pathfind_result p1, pathfind_result p2);
static bool is_path_in_bounds(Line path, Boundary bounds);
static bool is_point_in_bounds(Point p, Boundary bounds);

/* Miscellaneous functions */
static MultiPolygon circle_from_obstacle(obstacle o, double extra_buffer);
static double get_obstacle_buffer_size(void);


/**
 * @brief the pathfind() function is the core offering of this libpathfinding library
 * see pathfinding.h and the README.md for details
 */
vector<pathfind_result> pathfind(Boundary &bounds, vector<Point> &agents, vector<Point> &targets, vector<obstacle> &obstacles)
{
   vector<pathfind_result> final_results; // results vector

   // just print error and exit if inputs not valid
   if (!is_valid_input_params(bounds, agents, targets, obstacles))
   {
      throw std::invalid_argument("Invalid input parameters");
   }

   // iterate over each target, find the closest agent to assign to each target
   //
   // in my solution there is an implied hierarchy of targets-
   // by this method, we ensure target1 gets its closest agent, then target2 gets
   // its (next) closest agent, and so on.
   int id = 0; // unique ID for pathfinding result
   for (auto target : targets)
   {
      // generate agent bids for each agent
      vector<agent_bids> bids = {};

      cout << "\tTarget_" << id << " bidding opens" << endl;
      if (agents.empty())
      {
         cout << "\t\tWARNING: No agents left, remaining targets will not get paths" << endl;
         break;
      }

      for (vector<Point>::iterator it = agents.begin(); it != agents.end(); ++it)
      {
         size_t agent_id = std::distance(agents.begin(), it);

         // construct a bid for this agent and append to vector of bids
         Line chosen_path = calculate_path(bounds, *it, target, obstacles);
         agent_bids bid = {agent_id, *it, chosen_path, bg::length(chosen_path)};
         bids.push_back(bid);
      }

      // setup pathfinding result
      pathfind_result iter_result = {
          .id = id,
          .target = target,
      };

      // now choose best bid for target, then remove
      // selected agent from pool
      double iter_distance = DBL_MAX; // instantiate to worst case value
      size_t selected_agent_idx;
      cout << "\tTarget_" << id << " has received all bids" << endl;
      for (auto bid : bids)
      {
         cout << "\t\tBid_" << bid.agent_vect_idx << " dist=" << bid.distance << ", path=" << LP_PRINT_GEOM(bid.path) << endl;
         if (bid.distance < iter_distance)
         {
            iter_distance = bid.distance;
            iter_result.agent = bid.agent;
            iter_result.path = bid.path;
            selected_agent_idx = bid.agent_vect_idx;
         }
      }
      // now lock in the choice and pope the agent
      cout << "\t\tTarget_" << id << " selects: bid_" << selected_agent_idx << endl;
      final_results.push_back(iter_result);

      if (agents.size() > 1)
      {
         agents.erase(agents.begin() + selected_agent_idx); // pop assigned agent
      }
      else
      {
         agents.clear();
      }
      id++;
   }

   // now all agents have been assigned, check for crossed paths
   cout << "\tPath plan is in, conducting final checks" << endl;
   bool is_crossing = true;

   size_t num_results = final_results.size();
   if (num_results > 1)
   {
      while (is_crossing)
      {
         is_crossing = false;

         // go through agent/ target assignments in reverse order that they were assigned 
         // reason for reverse order is that allocating in the forward order allowed for the cross
         for (int i=num_results-1; i >= 0; i--)
         {
            for (int j= num_results-1; j >= 0; j--)
            {
               if ((i != j) && (is_path_crossing(final_results.at(i), final_results.at(j))))
               {
                  is_crossing = true;
                  cout << "\t\tERROR: Paths [" << i << "," << j << "] are crossing - resolving" << endl;
                  swap_agents(final_results, i, j);
                  final_results.at(i).path = calculate_path(bounds, final_results.at(i).agent, final_results.at(i).target, obstacles);
                  final_results.at(j).path = calculate_path(bounds, final_results.at(j).agent, final_results.at(j).target, obstacles);
               }
            }
         }
      }
   }
   return final_results;
}

/**
 * @brief Create the curved line path that avoids obstacles for a single agent
 * Steps are: create union of straight_path and obstacles, get convex hull of union,
 * create resulting path from a subset of convex_hull points and start/end
 * @param straight_path a two-point line with {agent, target}
 * @param obstacles vector of all obstacles
 * @param is_clockwise true to reverse the convex_hull output before iterating
 */
static Line get_obstacle_avoid_path(Line straight_path, vector<obstacle> &obstacles, bool is_clockwise)
{
   boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(line_buffer_distance);
   boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
   boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
   boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
   boost::geometry::strategy::buffer::side_straight side_strategy;

   Line retval;
   MultiPolygon all_obstacles;

   /* stroke Line (series of points) into thin polygon */
   MultiPolygon line_buf;
   bg::buffer(straight_path, line_buf, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

   /**
    * iteratively create a polygon union of all obstacles intersecting with the straight line path
    * we expect to have at least one, or else we would have used pathfinding()'s straight_path
    */
   vector<obstacle> intersecting = get_intersecting_obstacles(straight_path, obstacles);
   for (auto shape : intersecting)
   {
      /**
       * create an ever-slightly-wider circle (see get_obstacle_buffer_size)
       * and stick it to our thin line_buf polygon
       */
      MultiPolygon circle = circle_from_obstacle(shape, get_obstacle_buffer_size());
      bg::union_(line_buf, circle[0], all_obstacles);
   }
   /* our most crucial trick, generate a convex hull line around the compound polygon */
   Line hull;
   bg::convex_hull(all_obstacles, hull);

   /**
    * now the rest of the algorithm is going to be about effectively
    * selecting a subset of the convex hull and making sure it exactly reaches our points
    */

   /* load the proper subset of the convex hull into the final path */
   Line convex_hull_subset = find_convex_hull_subset(straight_path[0], straight_path[1], hull, is_clockwise);

   /**
    * with infinite time I'd like to figure out why I have so much trouble with order of these points- but this works
    * in a nutshell- if it appears clear we hop from the first point to the far end of the convex hull and vice versa,
    * and thus this path crosses itself, just reverse it
    */
   if (bg::distance(convex_hull_subset[0], straight_path[0]) > bg::distance(convex_hull_subset[convex_hull_subset.size() - 1], straight_path[0]))
   {
      bg::reverse(convex_hull_subset);
   }

   /* load the first point into the final path */
   retval.push_back(straight_path[0]);

   /**
    * because we're making a convex hull around a stroked path, there are some annoying small points around the buffer near each endpoint
    * we have this keepout to filter out those noisy parts and just take the path around the obstacle(s)
    */
   for (auto point : convex_hull_subset)
   {
      // we want points that are just a little bit beyond the "stroke width" value we used to turn the line into a polygon
      // which basically turned
      //  0----------0
      // into
      //   ___________________
      //  /                   \
      //  \___________________/
      // so we ignore points around those ends
      if (!(bg::distance(point, straight_path[0]) < (line_buffer_distance + 0.01)) && !(bg::distance(point, straight_path[1]) < (line_buffer_distance + 0.01)))
      {
         retval.push_back(point);
      }
   }

   /* load the last point into the final path after the first and the hull */
   retval.push_back(straight_path[1]);
   return retval;
}

/**
 * @brief given a convex hull, agent, and target, find subset of points from agent to target
 * this involves finding closest point in convex hull to agent and target, and taking a subvector
 * @param agent the Point of the agent, first point in straight_path
 * @param target the Point of the final target, second and final point in straight_path
 * @param convex_hull a closed-shape convex hull that goes around but doesn't touch agent/target
 * @param is_clockwise true to reverse convex_hull before iterating
 * @return Line with the relevant portion of provided convex_hull for pathfinding
 */
static Line find_convex_hull_subset(Point agent, Point target, Line convex_hull, bool is_clockwise)
{
   double start_distance;
   double end_distance;
   double min_start_distance = DBL_MAX;
   double min_end_distance = DBL_MAX;
   size_t start_idx;
   size_t end_idx;
   Line result;

   /* handle clockwise/counterclockwise by optionally reversing vector of points */
   if (!is_clockwise)
   {
      bg::reverse(convex_hull);
   }

   /**
    * iterate around whole closed shape, checking each points distances
    * to agent and to target, keep a running tally of closest point to each
    */
   for (auto it = convex_hull.begin(); it != convex_hull.end(); ++it)
   {
      start_distance = bg::distance(*it, agent);
      if (start_distance < min_start_distance)
      {
         min_start_distance = start_distance;
         start_idx = std::distance(convex_hull.begin(), it);
      }
      end_distance = bg::distance(*it, target);
      if (end_distance < min_end_distance)
      {
         min_end_distance = end_distance;
         end_idx = std::distance(convex_hull.begin(), it);
      }
   }
   /**
    *  now that we have closest points, return subset of closed shape from min(start,end) to max(start,end)
    *  TODO this is suboptimal and it should be possible to reason out this min/max/ reverse issue
    */
   result.assign(convex_hull.begin() + min(start_idx, end_idx), convex_hull.begin() + max(start_idx, end_idx));
   return result;
}

/**
 * @brief return an ever-growing amount of keepout as extra buffer around shapes
 * Necessary when multiple agents want to circumvent the same obstacle(s) to reach their targets
 * This is fed into circle_from_obstacle's optional arg for get_obstacle_avoid_path()
 * @return configurable min_buffer value * incrementing buffer_offset var
 */
static double get_obstacle_buffer_size(void)
{
   return (min_keepout_buffer * buffer_offset++);
}

/**
 * @brief Turn an obstacle into a circular MultiPolygon
 * @param o obstacle to become a circle
 * @param extra_buffer optionally increase buffer around obstacle. See get_obstacle_buffer_size()
 * @return MultiPolygon circular polygon made of 16 evenly spaced points around o.p
 */
static MultiPolygon circle_from_obstacle(obstacle o, double extra_buffer = 0)
{
   const int points_per_circle = 16; // configurable num points around circle

   /**
    * We use the concept of a buffer around a point to create our circle
    * boost::geometry strategies are effectively options about how we will generate our circle
    */
   bg::strategy::buffer::join_round join_strategy(points_per_circle); // unused for obstacles
   bg::strategy::buffer::end_round end_strategy(points_per_circle); // unused for obstacles
   bg::strategy::buffer::side_straight side_strategy; // unused for obstacles

   /* make a circle with 16 points around a single point */
   bg::strategy::buffer::point_circle circle_strategy(points_per_circle);

   /* set buffer distance (obstacle radius + extra buffer) */
   bg::strategy::buffer::distance_symmetric<double> distance_strategy(o.radius + extra_buffer);

   MultiPolygon result;
   bg::buffer(o.p, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
   return result;
}

/**
 * @brief validate the input agents and targets. If this fails, pathfinding cannot proceed
 * @param bounds Outer boundary box
 * @param agents vector of all agents
 * @param targets vector of all targets
 * @param obstacles vector of all obstacles
 * @return true if input params are valid, else false
 */
bool is_valid_input_params(Boundary &bounds, vector<Point> &agents, vector<Point> &targets, vector<obstacle> &obstacles)
{
   /* Ensure bounds has some area */
   if (bg::get<bg::min_corner, 0>(bounds) == bg::get<bg::max_corner, 0>(bounds) ||
       bg::get<bg::min_corner, 1>(bounds) == bg::get<bg::max_corner, 1>(bounds))
   {
      cerr << "ERROR: Boundary box has either no height or width" << endl;
      return false;
   }

   /* ensure we don't exceed max number of agents */
   if (agents.size() > NUM_MAX_AGENTS)
   {
      cerr << "ERROR: Provided number of agents exceeds max value:" << NUM_MAX_AGENTS << endl;
      return false;
   }

   /* check agent validity */
   for (auto agent : agents)
   {
      // ensure all agents are inbounds
      if (!is_point_in_bounds(agent, bounds))
      {
         cerr << "ERROR: Agent located outside boundary" << endl;
         return false;
      }
      // ensure no agents are within obstacles
      for (auto obs : obstacles)
      {
         MultiPolygon circle = circle_from_obstacle(obs);
         if (bg::covered_by(agent, circle))
         {
            cerr << "ERROR: Agent located within obstacle" << endl;
            return false;
         }
      }
   }

   /* check target validity */
   for (auto target : targets)
   {
      // ensure all targets are inbounds
      if (!is_point_in_bounds(target, bounds))
      {
         cerr << "ERROR: Target located outside boundary" << endl;
         return false;
      }
      // ensure no targets are within obstacles
      for (auto obs : obstacles)
      {
         MultiPolygon circle = circle_from_obstacle(obs);
         if (bg::covered_by(target, circle))
         {
            cerr << "ERROR: Target located within obstacle" << endl;
            return false;
         }
      }
   }

   /* ensure no obstacles contain the box */
   for (auto obs : obstacles)
   {
      MultiPolygon circle = circle_from_obstacle(obs);
      if (bg::covered_by(bounds, circle))
      {
         cerr << "ERROR: Whole boundary within obstacle" << endl;
         return false;
      }
   }

   /* ensure no obstacles bifurcate or intersect the box */
   for (auto obs : obstacles)
   {
      MultiPolygon mp;
      MultiPolygon circle = circle_from_obstacle(obs);
      bg::difference(bounds, circle, mp);

      if (bg::num_geometries(mp) > 1)
      {
         cerr << "ERROR: Obstacle bifurcates or intersects the boundary" << endl;
         return false;
      }
   }
   return true;
}

/**
 * @brief exchange agents between two final pathfinding results
 * Throws std::out_of_range if idx_1 or idx_2 >= size()
 * @param pr the full pathfinding results
 * @param idx_1 the first index into pr to swap
 * @param idx_2 the second index into pr to swap
 */
static void swap_agents(vector<pathfind_result> &pr, int idx_1, int idx_2)
{
   Point temp_agent = pr.at(idx_1).agent;
   pr.at(idx_1).agent = pr.at(idx_2).agent;
   pr.at(idx_2).agent = temp_agent;
}

/**
 * @brief Calculate a path from agent to target, this is sort of a state machine
 * that selects straight or curved path, and then orchestrates curved path design if necessary
 * @param bounds outer bounding Box
 * @param agent agent that must route to target
 * @param target target to be routed to
 * @param obstacles vector of circular obstacles to avoid
 * @return a new straight or curved path from agent to target
 */
static Line calculate_path(Boundary &bounds, Point agent, Point target, vector<obstacle> &obstacles)
{
   /* check how many obstacles are intersecting */
   Line straight_path = {Point(agent.x(), agent.y()), Point(target.x(), target.y())};
   vector<obstacle> intersecting = get_intersecting_obstacles(straight_path, obstacles);

   /**
    * Easy case: a straight line to the target will always be the
    * best bid for a particular agent if it is avaialable
    */
   if (intersecting.empty() &&
       is_path_in_bounds(straight_path, bounds))
   {
      cout << "\t\t\tpath will be straight line" << endl;
      // return the straight path
      return straight_path;
   }
   /**
    * Hard case: if even one obstacle intersects the path, 
    * curved object-avoiding path is needed
    */
   else if (!intersecting.empty())
   {
      cout << "\t\t\tpath will be convex hull" << endl;
      /* Attempt clockwise object-avoiding path */
      Line curved_path = get_obstacle_avoid_path(straight_path, obstacles, true);
      // confession- after messing with this, I am not certain clockwise_arg is truly clockwise
      if (!is_path_in_bounds(curved_path, bounds))
      {
         cout << "\t\t\t\tWARNING: clockwise path is OOB - trying counterclockwise" << endl;
         /* first try was out of bounds, reverse path and try again  */
         curved_path = get_obstacle_avoid_path(straight_path, obstacles, false);
         if (!is_path_in_bounds(curved_path, bounds))
         {
            throw runtime_error("ERROR: Agent reports no way around obstacle");
         }
      }
      // return the curved path
      return curved_path;
   }
   else
   {
      // should not be possible
      throw runtime_error("ERROR: Agent or target is out of bounds, initial checks insufficient");
   }
}

/**
 * @brief test whether two paths intersect
 * @param p1 one pathfinding_result, from which path will be obtained
 * @param p2 another pathfinding_result, from which path will be obtained
 * @return true if p1.path and p2.path intersect, else false
 */
static bool is_path_crossing(pathfind_result p1, pathfind_result p2)
{
   return bg::intersects(p1.path, p2.path);
}

/**
 * @brief Test whether any point in a path ever leaves the boundary
 * @param path path under test
 * @param bounds outer boundary Box, all points along p should be inside this
 * @return true if path between {target,agent} never leaves the boundary, else false
 */
static bool is_path_in_bounds(Line path, Boundary bounds)
{
   bool retval = true;
   for (auto point : path)
   {
      if (!is_point_in_bounds(point, bounds))
      {
         retval = false;
      }
   }
   return retval;
}

/**
 * @brief test whether a point is in bounds
 * @param p point under test
 * @param bounds outer boundary Box point p should be inside this
 * @return true if path never leaves the boundary, else false
 */
static bool is_point_in_bounds(Point p, Boundary bounds)
{
   return bg::covered_by(p, bounds);
}

/**
 * @brief Test whether a provided path crosses one or more obstacles
 * @param path Line from target to agent to check for obstacles
 * @param obstacles A vector of circular obstacles
 * @return vector of intersecting obstacles, empty if no intersection
 */
static vector<obstacle> get_intersecting_obstacles(Line path, vector<obstacle> obstacles)
{
   vector<obstacle> intersecting = {};

   // Check whether any obstacle intersects with this segment
   for (auto obs : obstacles)
   {
      MultiPolygon result = circle_from_obstacle(obs);
      /**
       * rather than check whether every point or n-many points along the line
       * is within a particular radius, we instead create a circle as a keepout buffer around a point
       * and then use the boost::geometry intersection checker
       */
      if (bg::intersects(path, result))
      {
         intersecting.push_back(obs);
      }
   }
   return intersecting;
}

void print_result(Boundary &bounds, vector<obstacle> &obstacles, vector<pathfind_result> &results)
{
   /**
    * The associated python rendering script render_result.py is designed to ignore blank lines 
    * and lines that begin with tab so that output from this library can be read 
    * for diagnostic information or simply piped to a .csv to render with the python script
    */

   // print CSV Header
   cout << endl;
   cout << "type,node_idx,agent_x,agent_y,target_x,target_y,";
   cout << "path,";
   cout << "obstacle_x,obstacle_y,obstacle_rad,";
   cout << "boundary_x0,boundary_x1,boundary_y0,boundary_y1";
   cout << endl;

   // print outer boundary
   double x_0 = bg::get<bg::min_corner, 0>(bounds);
   double x_1 = bg::get<bg::max_corner, 0>(bounds);
   double y_0 = bg::get<bg::min_corner, 1>(bounds);
   double y_1 = bg::get<bg::max_corner, 1>(bounds);
   cout << "2," << ",,,,,,,,," << x_0 << "," << x_1 << "," << y_0 << "," << y_1 << endl;

   // print pathfinding vectors
   for (auto result : results)
   {
      cout << "1," << result.id << ",";
      cout << result.agent.x() << "," << result.agent.y() << ",";
      cout << result.target.x() << "," << result.target.y() << ",";
      cout << "\"" << LP_PRINT_GEOM(result.path) << "\",";
      cout << ",,,," << endl;
   }

   // print obstacles
   for (auto obs : obstacles)
   {
      cout << "3," << ",,,,,,";
      cout << obs.p.x() << "," << obs.p.y() << "," << obs.radius;
      cout << ",,,," << endl;
   }
}
