/**
 * *********************************************************
 *
 * @file: informed_rrt_star.h
 * @brief: Contains the Informed RRT* planner class, ported from
 *         https://github.com/thisisjaskaran/informed-rrt-star
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef INFORMED_RRT_STAR_H
#define INFORMED_RRT_STAR_H

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the Informed RRT* algorithm as
 *        implemented by https://github.com/thisisjaskaran/informed-rrt-star
 *
 *        The reference implementation runs in two phases:
 *          1) grow a plain RRT*, sampling uniformly over the whole map, until
 *             a new node lands within `search_radius_` of the goal and has a
 *             collision-free line of sight to it;
 *          2) once connected, refine the solution for a fixed number of
 *             iterations by sampling only inside the informed ellipse defined
 *             by the current best cost, still growing/rewiring the same RRT*
 *             tree.
 *
 *        Both phases share the same nearest-node / steer / rewire pipeline,
 *        matching the original `map.py` mechanism as closely as C++ and this
 *        package's `Node`/costmap conventions allow.
 */
class InformedRRTStar : public GlobalPlanner
{
public:
  /**
   * @brief  Constructor
   * @param   costmap               the environment for path planning
   * @param   sample_num            phase-1 sampling budget (safety bound; the
   *                                 reference script loops unconditionally
   *                                 until the goal is reached)
   * @param   step_size             steer step size (`step_size` upstream)
   * @param   search_radius         rewire / goal-connection radius (`search_radius` upstream)
   * @param   informed_iterations   phase-2 refinement iterations (`ITERATIONS` upstream)
   * @param   informed_threshold_cost early-stop cost for phase-2 (`threshold_cost` upstream)
   */
  InformedRRTStar(costmap_2d::Costmap2D* costmap, int sample_num, double step_size, double search_radius,
                   int informed_iterations, double informed_threshold_cost);

  /**
   * @brief Informed RRT* implementation
   * @param start     start node
   * @param goal      goal node
   * @param path      optimal path consists of Node
   * @param expand    containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Uniformly sample a random node over the whole map (`Map.sample`)
   * @return sampled node
   */
  Node _uniformSample();

  /**
   * @brief Sample a random node inside the informed ellipse (`Map.informed_sample`)
   * @param c_best  current best path cost
   * @return sampled node
   */
  Node _informedSample(double c_best);

  /**
   * @brief Find the nearest node to `x_rand` currently in the tree (`Map.nearest_node`)
   * @param x_rand  sampled node
   * @return nearest node
   */
  Node _nearestNode(const Node& x_rand);

  /**
   * @brief Steer from `x_nearest` towards `x_rand` by at most `step_size_` (`Map.steer`)
   * @param x_nearest nearest node in the tree
   * @param x_rand    sampled node
   * @return steered node
   */
  Node _steer(const Node& x_nearest, const Node& x_rand);

  /**
   * @brief Check whether a node lies inside the costmap bounds (`Map.is_valid`)
   */
  bool _isValid(const Node& node);

  /**
   * @brief Check whether a node lies on an obstacle cell (`Map.is_in_obstacle`)
   */
  bool _isInObstacle(const Node& node);

  /**
   * @brief Check whether the straight segment between 2 nodes is free of
   *        obstacles, sampling it at 10 fixed fractions (`Map.collision_free`)
   */
  bool _collisionFree(const Node& n1, const Node& n2);

  /**
   * @brief Collect the tree nodes within `search_radius_` of `x_new` that can
   *        be connected to it without collision (`Map.get_nodes_in_radius`)
   */
  std::vector<Node> _nodesInRadius(const Node& x_new);

  /**
   * @brief Connect `x_new` to the cheapest neighbor in `nodes_in_radius`, add
   *        it to the tree, then try to rewire the remaining neighbors through
   *        it (`Map.rewire`)
   * @param x_new             candidate node (parent/cost filled in on success)
   * @param nodes_in_radius   neighbor nodes gathered by `_nodesInRadius`
   * @return true if `x_new` was connected and inserted into the tree
   */
  bool _rewire(Node& x_new, std::vector<Node>& nodes_in_radius);

  /**
   * @brief Walk the goal's parent chain to compute the current best path cost
   *        (`Map.get_best_cost`)
   */
  double _getBestCost();

protected:
  Node start_, goal_;                                   // start and goal node copy
  std::unordered_map<int, Node> sample_list_;           // set of sample nodes, keyed by grid index
  std::vector<int> insertion_order_;                    // ids in the order they were added to the tree

  int sample_num_;                    // phase-1 sampling budget (safety bound)
  double step_size_;                  // steer step size
  double search_radius_;              // rewire / goal-connection radius
  int informed_iterations_;           // phase-2 refinement iterations
  double informed_threshold_cost_;    // early-stop cost threshold for phase-2
};
}  // namespace global_planner
#endif  // INFORMED_RRT_STAR_H
