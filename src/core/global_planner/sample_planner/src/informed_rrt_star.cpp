/**
 * *********************************************************
 *
 * @file: informed_rrt_star.cpp
 * @brief: Contains the Informed RRT* planner class, ported from
 *         https://github.com/thisisjaskaran/informed-rrt-star
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>
#include <random>
#include <limits>

#include <Eigen/Dense>

#include "informed_rrt_star.h"

namespace global_planner
{
/**
 * @brief  Constructor
 */
InformedRRTStar::InformedRRTStar(costmap_2d::Costmap2D* costmap, int sample_num, double step_size,
                                  double search_radius, int informed_iterations, double informed_threshold_cost)
  : GlobalPlanner(costmap)
  , sample_num_(sample_num)
  , step_size_(step_size)
  , search_radius_(search_radius)
  , informed_iterations_(informed_iterations)
  , informed_threshold_cost_(informed_threshold_cost)
{
}

/**
 * @brief Informed RRT* implementation
 */
bool InformedRRTStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  path.clear();
  expand.clear();
  sample_list_.clear();
  insertion_order_.clear();

  start_ = start;
  goal_ = goal;
  sample_list_.insert(std::make_pair(start_.id(), start_));
  insertion_order_.push_back(start_.id());
  expand.push_back(start_);

  bool first_sample = true;
  bool goal_connected = false;
  Node x_new = start_;

  // ---------- phase 1: plain RRT*, uniform sampling, until close enough to the goal ----------
  int iteration = 0;
  while (iteration < sample_num_)
  {
    iteration++;

    Node x_rand = _uniformSample();
    Node x_nearest = _nearestNode(x_rand);
    Node x_candidate = _steer(x_nearest, x_rand);

    if (!_isValid(x_candidate) || _isInObstacle(x_candidate))
      continue;

    std::vector<Node> nodes_in_radius = _nodesInRadius(x_candidate);

    bool added;
    if (first_sample)
    {
      // reference implementation attaches the very first accepted sample
      // directly to the start node, bypassing the rewire step
      x_candidate.set_pid(start_.id());
      x_candidate.set_g(helper::dist(x_candidate, start_));
      sample_list_.insert(std::make_pair(x_candidate.id(), x_candidate));
      insertion_order_.push_back(x_candidate.id());
      first_sample = false;
      added = true;
    }
    else
      added = _rewire(x_candidate, nodes_in_radius);

    if (!added)
      continue;

    expand.push_back(x_candidate);
    x_new = x_candidate;

    if (helper::dist(x_new, goal_) < search_radius_ && _collisionFree(x_new, goal_))
    {
      goal_connected = true;
      break;
    }
  }

  if (!goal_connected)
    return false;

  Node goal_node = goal_;
  goal_node.set_pid(x_new.id());
  goal_node.set_g(x_new.g() + helper::dist(x_new, goal_));
  sample_list_.insert(std::make_pair(goal_node.id(), goal_node));
  insertion_order_.push_back(goal_node.id());

  // ---------- phase 2: refine the path, sampling inside the shrinking informed ellipse ----------
  for (int i = 0; i < informed_iterations_; i++)
  {
    if (_getBestCost() < informed_threshold_cost_)
      break;

    Node x_rand = _informedSample(_getBestCost());
    Node x_nearest = _nearestNode(x_rand);
    Node x_candidate = _steer(x_nearest, x_rand);

    if (!_isValid(x_candidate) || _isInObstacle(x_candidate))
      continue;

    std::vector<Node> nodes_in_radius = _nodesInRadius(x_candidate);
    if (_rewire(x_candidate, nodes_in_radius))
      expand.push_back(x_candidate);
  }

  path = _convertClosedListToPath(sample_list_, start_, goal_);
  return !path.empty();
}

/**
 * @brief Uniformly sample a random node over the whole map (`Map.sample`)
 */
Node InformedRRTStar::_uniformSample()
{
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_int_distribution<int> dist_x(0, static_cast<int>(costmap_->getSizeInCellsX()) - 1);
  std::uniform_int_distribution<int> dist_y(0, static_cast<int>(costmap_->getSizeInCellsY()) - 1);

  int x = dist_x(eng);
  int y = dist_y(eng);
  return Node(x, y, 0, 0, grid2Index(x, y), 0);
}

/**
 * @brief Sample a random node inside the informed ellipse (`Map.informed_sample`)
 */
Node InformedRRTStar::_informedSample(double c_best)
{
  double c_min = helper::dist(start_, goal_);

  Eigen::Vector3d x_centre((start_.x() + goal_.x()) / 2.0, (start_.y() + goal_.y()) / 2.0, 0.0);

  Eigen::Vector3d a1((goal_.x() - start_.x()) / c_min, (goal_.y() - start_.y()) / c_min, 0.0);
  Eigen::RowVector3d i1(1.0, 0.0, 0.0);
  Eigen::Matrix3d M = a1 * i1;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& U = svd.matrixU();
  const Eigen::Matrix3d& V = svd.matrixV();

  Eigen::Matrix3d c_middle_term = Eigen::Matrix3d::Identity();
  c_middle_term(2, 2) = U.determinant() * V.determinant();
  Eigen::Matrix3d C = U * c_middle_term * V.transpose();

  double under_sqrt = c_best * c_best - c_min * c_min;
  double diag_terms = under_sqrt >= 0.0 ? std::sqrt(under_sqrt) / 2.0 : c_best / 2.0;

  Eigen::Matrix3d L = Eigen::Matrix3d::Identity() * diag_terms;
  L(0, 0) = c_best / 2.0;

  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  double theta = uni(eng) * 2.0 * M_PI;
  double radius = uni(eng);
  Eigen::Vector3d x_ball(radius * std::cos(theta), radius * std::sin(theta), 0.0);

  Eigen::Vector3d x_f = C * L * x_ball + x_centre;

  int x = static_cast<int>(x_f(0));
  int y = static_cast<int>(x_f(1));
  return Node(x, y, 0, 0, grid2Index(x, y), 0);
}

/**
 * @brief Find the nearest node to `x_rand` currently in the tree (`Map.nearest_node`)
 */
Node InformedRRTStar::_nearestNode(const Node& x_rand)
{
  Node nearest = sample_list_.begin()->second;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& kv : sample_list_)
  {
    double d = helper::dist(kv.second, x_rand);
    if (d < min_dist)
    {
      min_dist = d;
      nearest = kv.second;
    }
  }
  return nearest;
}

/**
 * @brief Steer from `x_nearest` towards `x_rand` by at most `step_size_` (`Map.steer`)
 */
Node InformedRRTStar::_steer(const Node& x_nearest, const Node& x_rand)
{
  double d = helper::dist(x_nearest, x_rand);
  if (d <= step_size_)
    return Node(x_rand.x(), x_rand.y(), 0, 0, grid2Index(x_rand.x(), x_rand.y()), 0);

  int x = x_nearest.x() + static_cast<int>((x_rand.x() - x_nearest.x()) * step_size_ / d);
  int y = x_nearest.y() + static_cast<int>((x_rand.y() - x_nearest.y()) * step_size_ / d);
  return Node(x, y, 0, 0, grid2Index(x, y), 0);
}

/**
 * @brief Check whether a node lies inside the costmap bounds (`Map.is_valid`)
 */
bool InformedRRTStar::_isValid(const Node& node)
{
  return node.x() >= 0 && node.x() < static_cast<int>(costmap_->getSizeInCellsX()) && node.y() >= 0 &&
         node.y() < static_cast<int>(costmap_->getSizeInCellsY());
}

/**
 * @brief Check whether a node lies on an obstacle cell (`Map.is_in_obstacle`)
 */
bool InformedRRTStar::_isInObstacle(const Node& node)
{
  return costmap_->getCharMap()[node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_;
}

/**
 * @brief Check whether the straight segment between 2 nodes is free of
 *        obstacles, sampling it at 10 fixed fractions (`Map.collision_free`)
 */
bool InformedRRTStar::_collisionFree(const Node& n1, const Node& n2)
{
  if (_isInObstacle(n1) || _isInObstacle(n2))
    return false;

  const int parts = 10;
  for (int i = 1; i < parts; i++)
  {
    int x = static_cast<int>((n1.x() * i + n2.x() * (parts - i)) / static_cast<double>(parts));
    int y = static_cast<int>((n1.y() * i + n2.y() * (parts - i)) / static_cast<double>(parts));
    if (costmap_->getCharMap()[grid2Index(x, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_)
      return false;
  }
  return true;
}

/**
 * @brief Collect the tree nodes within `search_radius_` of `x_new` that can
 *        be connected to it without collision (`Map.get_nodes_in_radius`)
 */
std::vector<Node> InformedRRTStar::_nodesInRadius(const Node& x_new)
{
  std::vector<Node> result;
  for (int id : insertion_order_)
  {
    const Node& node = sample_list_.at(id);
    if (helper::dist(node, x_new) <= search_radius_ && _collisionFree(node, x_new))
      result.push_back(node);
  }
  return result;
}

/**
 * @brief Connect `x_new` to the cheapest neighbor in `nodes_in_radius`, add it
 *        to the tree, then try to rewire the remaining neighbors through it
 *        (`Map.rewire`)
 */
bool InformedRRTStar::_rewire(Node& x_new, std::vector<Node>& nodes_in_radius)
{
  // recompute every node's cost from its current parent, in the order nodes
  // were added to the tree, mirroring the reference implementation's
  // per-iteration cost refresh over `self.nodes`
  for (int id : insertion_order_)
  {
    Node& node = sample_list_.at(id);
    if (node == start_)
    {
      node.set_g(0.0);
      continue;
    }
    auto parent_it = sample_list_.find(node.pid());
    if (parent_it != sample_list_.end())
      node.set_g(parent_it->second.g() + helper::dist(node, parent_it->second));
  }

  // connect to the cheapest neighbor
  double best_cost = std::numeric_limits<double>::max();
  int best_id = -1;
  for (const Node& neighbor : nodes_in_radius)
  {
    auto it = sample_list_.find(neighbor.id());
    if (it == sample_list_.end())
      continue;
    double cost = it->second.g() + helper::dist(it->second, x_new);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_id = neighbor.id();
    }
  }

  if (best_id == -1)
    return false;

  Node best_parent = sample_list_.at(best_id);
  x_new.set_pid(best_parent.id());
  x_new.set_g(helper::dist(best_parent, x_new) + best_parent.g());
  sample_list_.insert(std::make_pair(x_new.id(), x_new));
  insertion_order_.push_back(x_new.id());

  // try to rewire the remaining neighbors through the newly inserted node
  for (const Node& neighbor : nodes_in_radius)
  {
    if (neighbor.id() == best_id)
      continue;
    auto it = sample_list_.find(neighbor.id());
    if (it == sample_list_.end() || !_collisionFree(x_new, it->second))
      continue;
    double new_cost = x_new.g() + helper::dist(x_new, it->second);
    if (new_cost < it->second.g())
    {
      it->second.set_pid(x_new.id());
      it->second.set_g(new_cost);
    }
  }

  return true;
}

/**
 * @brief Walk the goal's parent chain to compute the current best path cost
 *        (`Map.get_best_cost`)
 */
double InformedRRTStar::_getBestCost()
{
  auto it = sample_list_.find(goal_.id());
  if (it == sample_list_.end())
    return std::numeric_limits<double>::max();

  double cost = 0.0;
  Node current = it->second;
  while (!(current == start_))
  {
    auto parent_it = sample_list_.find(current.pid());
    if (parent_it == sample_list_.end())
      break;
    cost += helper::dist(current, parent_it->second);
    current = parent_it->second;
  }
  return cost;
}
}  // namespace global_planner
