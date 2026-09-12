/**
 * *********************************************************
 *
 * @file: rrt.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2022-10-27
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>
#include <random>

#include "rrt_astar.h"

namespace global_planner
{
/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 */
RRTAStar::RRTAStar(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, int k)
  : RRT(costmap, sample_num, max_dist), k_(k)
{
}

/**
 * @brief RRT implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTAStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  path.clear();
  expand.clear();

  sample_list_.clear();
  // copy
  start_ = start, goal_ = goal;
  sample_list_.insert(std::make_pair(start.id(), start));
  expand.push_back(start);

  // extra 
  // int k = 5; // k sample point
  std::vector<double> f_values;
  std::vector<Node> nearest_nodes, rand_nodes;

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  { 
    f_values.clear();
    nearest_nodes.clear();
    rand_nodes.clear();

    // k loops
    for (int i = 0; i < k_; i++) {

      Node rand_node = _generateRandomNode();
      // obstacle
      if (costmap_->getCharMap()[rand_node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_)
        continue;
      // visited
      if (sample_list_.find(rand_node.id()) != sample_list_.end())
        continue;
      
      Node nearest_node = _getNearestNode(sample_list_, rand_node);

      // Calculate f_cost
      double g_cost = nearest_node.g();
      double h_cost = helper::dist(nearest_node, goal_);
      double f_cost = g_cost + h_cost;
      
      // Store for comparison
      f_values.push_back(f_cost);
      nearest_nodes.push_back(nearest_node);
      rand_nodes.push_back(rand_node);

    }

    Node best_nearest_node, best_rand_node;

    if (!f_values.empty()) {

      auto min_it = std::min_element(f_values.begin(), f_values.end());
      int min_idx = std::distance(f_values.begin(), min_it);

      best_nearest_node = nearest_nodes[min_idx];
      best_rand_node = rand_nodes[min_idx];

    } else {
    continue;
    }

    //add node
    Node new_node = _getNewNode(best_nearest_node, best_rand_node);
    if (new_node.id() == -1)
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.push_back(new_node);
    }

    // goal found
    if (_checkGoal(new_node))
    {
      path = _convertClosedListToPath(sample_list_, start, goal);
      return true;
    }
    iteration++;
  }
  return false;
}

/**
 * @brief Get nearest node by Nam
 * @param list  samplee list
 * @param node  sample node
 * @return nearest node
 */
Node RRTAStar::_getNearestNode(std::unordered_map<int, Node>& list, const Node& node)
{
  Node nearest_node;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : list)
  {
    // calculate distance
    double new_dist = helper::dist(p.second, node);

    // update nearest node
    if (new_dist < min_dist)
    {
      nearest_node = p.second;
      min_dist = new_dist;
    }
  }
  return nearest_node;
}

/**
 * @brief Get new node by Nam
 * @param nearest_node  nearest node in sample list
 * @param node  sample node
 * @return nearest node
 */
Node RRTAStar::_getNewNode(Node& nearest_node, const Node& node)
{
  Node new_node(node);
  double new_dist = helper::dist(nearest_node, new_node);

  // if distance longer than threshold
  if(new_dist > max_dist_)
  {
    double theta = helper::angle(nearest_node, new_node);
    new_node.set_x(nearest_node.x() + static_cast<int>(max_dist_ * cos(theta)));
    new_node.set_y(nearest_node.y() + static_cast<int>(max_dist_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_pid(nearest_node.id());
    new_node.set_g(max_dist_ + nearest_node.g());
  }else
  { 
    new_node.set_pid(nearest_node.id());
    new_node.set_g(new_dist + nearest_node.g());
  }

  // obstacle
  if (_isAnyObstacleInPath(new_node, nearest_node))
    new_node.set_id(-1);

  return new_node;
}

// double RRTAStar::heuristic(const Node& n1, const Node& n2){
//   return std::abs(n1.x() - n2.x()) + std::abs(n1.y() - n2.y());
// }
}  // namespace global_planner
