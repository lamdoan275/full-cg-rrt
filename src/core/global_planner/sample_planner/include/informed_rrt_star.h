/**
 * *********************************************************
 *
 * @file: informed_rrt_star.h
 * @brief: Contains the improved Informed RRT* global planner class
 * @author: Junting Hou, Linzhen Shi, Wensong Jiang, Zai Luo, Li Yang
 * @date: 2024
 * @version: 1.0
 *
 * Copyright (c) 2024, Junting Hou, Linzhen Shi, Wensong Jiang, Zai Luo, Li Yang.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef INFORMED_RRT_STAR_H
#define INFORMED_RRT_STAR_H

#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Global-planning part of the improved Informed RRT* described in
 *        "Dynamic path planning of mobile robots by combining improved
 *        informed-RRT* and VFH+ algorithms".
 *
 * Stage 1 alternately grows trees rooted at start and goal.  The active tree
 * is biased towards the last node, or another node, of the opposite tree.
 * Stage 2 keeps the two trees and refines the first solution by sampling the
 * informed ellipse.  Both stages use the paper's APF-based adaptive step.
 */
class InformedRRTStar : public GlobalPlanner
{
public:
  InformedRRTStar(costmap_2d::Costmap2D* costmap, int sample_num, double max_step, double search_radius,
                  int informed_iterations, double informed_threshold_cost, double goal_bias_probability,
                  double min_step, double danger_distance, double near_distance, double attractive_gain,
                  double repulsive_gain, double obstacle_influence_distance);

  /**
   * @brief Plan a path. The returned vector follows the package convention:
   *        goal to start (SamplePlanner reverses it when creating a ROS path).
   *
   * Fig. 8 of the paper re-enters the global planner only through the
   * "Is the current path reachable? -> No -> Update environment information"
   * branch, so a stored path is returned unchanged while it stays valid.
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * @brief Current local sub-goal of Sec. 3.6.  It is the next key node the
   *        robot has not reached yet and only advances on arrival.
   * @param sub_goal  filled with the current sub-goal
   * @return false while no plan exists
   */
  bool subGoal(Node& sub_goal) const;

  /**
   * @brief Key nodes of the stored plan, ordered start -> goal.
   */
  const std::vector<Node>& keyNodes() const
  {
    return key_nodes_;
  }

protected:
  struct Tree
  {
    std::unordered_map<int, Node> nodes;
    std::vector<int> insertion_order;
    int root_id;
    int last_id;

    Tree() : root_id(-1), last_id(-1)
    {
    }
  };

  void _resetTree(Tree& tree, const Node& root);

  Node _uniformSample();
  Node _informedSample(double c_best);
  Node _biasedTarget(const Tree& opposite_tree);
  Node _randomOppositeNode(const Tree& opposite_tree);

  Node _nearestNode(const Tree& tree, const Node& target) const;
  Node _steer(const Node& nearest, const Node& target, double step) const;
  bool _extendTree(Tree& tree, const Node& target, const Node& attractive_root, Node& new_node);
  bool _insertAndRewire(Tree& tree, Node& new_node, const Node& nearest);

  bool _tryConnect(bool active_is_start, const Node& new_node);
  void _considerConnection(int start_tree_id, int goal_tree_id);
  std::vector<Node> _buildPath(int start_tree_id, int goal_tree_id) const;
  std::vector<Node> _traceToRoot(const Tree& tree, int node_id) const;
  std::vector<Node> _extractKeyNodes(const std::vector<Node>& raw_path) const;

  /**
   * @brief Interpolate a key-node polyline down to one-cell spacing.  VFH+
   *        generates the motion between two sub-goals in the paper; here the
   *        local planner does, and it needs poses inside its rolling window.
   */
  std::vector<Node> _densify(const std::vector<Node>& polyline) const;

  /** @brief "Is the current path reachable?" of Fig. 8. */
  bool _cachedPlanUsable(const Node& start, const Node& goal) const;
  void _resetSubGoals(const std::vector<Node>& key_nodes_goal_to_start);
  void _advanceSubGoal(const Node& start);

  bool _isAncestor(const Tree& tree, int ancestor_id, int node_id) const;
  void _updateDescendantCosts(Tree& tree, int parent_id, std::unordered_set<int>& visited);

  bool _isValid(const Node& node) const;
  bool _isInObstacle(const Node& node) const;
  bool _collisionFree(const Node& n1, const Node& n2) const;

  void _buildClearanceMap();
  double _obstacleClearance(const Node& node) const;
  double _adaptiveStep(const Node& sample, const Node& attractive_root) const;

  double _random01();

protected:
  Node start_;
  Node goal_;
  Tree start_tree_;
  Tree goal_tree_;

  int sample_num_;
  double max_step_;                     // grid cells
  double search_radius_;                // grid cells
  int informed_iterations_;
  double informed_threshold_cost_;      // grid cells

  double goal_bias_probability_;
  double min_step_;                     // grid cells
  double danger_distance_;              // metres
  double near_distance_;                // metres
  double attractive_gain_;
  double repulsive_gain_;
  double obstacle_influence_distance_;  // metres

  double c_best_;
  std::vector<Node> best_path_;
  std::vector<double> clearance_map_;    // distance to nearest obstacle, grid cells

  // Plan kept between calls so that the path, and therefore the sub-goal
  // sequence, only changes when the paper says it should.
  bool has_plan_;
  int cached_goal_id_;
  std::vector<Node> cached_path_;        // densified, goal -> start
  std::vector<Node> cached_expand_;      // tree of the last real planning run
  std::vector<Node> key_nodes_;          // key nodes, start -> goal
  std::size_t sub_goal_index_;           // index of the current sub-goal in key_nodes_

  std::mt19937 rng_;
};
}  // namespace global_planner

#endif  // INFORMED_RRT_STAR_H
