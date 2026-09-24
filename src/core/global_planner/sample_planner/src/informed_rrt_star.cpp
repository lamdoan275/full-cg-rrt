/**
 * *********************************************************
 *
 * @file: informed_rrt_star.cpp
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
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include <Eigen/Dense>

#include "informed_rrt_star.h"

namespace global_planner
{
namespace
{
// Sec. 3.6
const double kSubGoalToleranceMetres = 0.5;


const double kMaxDeviationMetres = 1.5;
}  // namespace

InformedRRTStar::InformedRRTStar(costmap_2d::Costmap2D* costmap, int sample_num, double max_step,
                                 double search_radius, int informed_iterations, double informed_threshold_cost,
                                 double goal_bias_probability, double min_step, double danger_distance,
                                 double near_distance, double attractive_gain, double repulsive_gain,
                                 double obstacle_influence_distance)
  : GlobalPlanner(costmap)
  , sample_num_(sample_num)
  , max_step_(std::max(1.0, max_step))
  , search_radius_(std::max(1.0, search_radius))
  , informed_iterations_(informed_iterations)
  , informed_threshold_cost_(informed_threshold_cost)
  , goal_bias_probability_(std::max(0.0, std::min(1.0, goal_bias_probability)))
  , min_step_(std::max(1.0, std::min(min_step, max_step_)))
  , danger_distance_(std::max(0.0, danger_distance))
  , near_distance_(std::max(danger_distance_, near_distance))
  , attractive_gain_(std::max(0.0, attractive_gain))
  , repulsive_gain_(std::max(0.0, repulsive_gain))
  , obstacle_influence_distance_(std::max(near_distance_, obstacle_influence_distance))
  , c_best_(std::numeric_limits<double>::infinity())
  , has_plan_(false)
  , cached_goal_id_(-1)
  , sub_goal_index_(0)
  , rng_(std::random_device()())
{
}

bool InformedRRTStar::plan(const Node& start, const Node& goal, std::vector<Node>& path,
                           std::vector<Node>& expand)
{
  path.clear();
  expand.clear();

  // Fig. 8 
  if (_cachedPlanUsable(start, goal))
  {
    _advanceSubGoal(start);
    path = cached_path_;
    expand = cached_expand_;
    return true;
  }

  best_path_.clear();
  cached_path_.clear();
  cached_expand_.clear();
  key_nodes_.clear();
  sub_goal_index_ = 0;
  has_plan_ = false;
  c_best_ = std::numeric_limits<double>::infinity();

  start_ = start;
  goal_ = goal;
  _resetTree(start_tree_, start_);
  _resetTree(goal_tree_, goal_);
  expand.push_back(start_);
  expand.push_back(goal_);

  _buildClearanceMap();

  // Phase 1:
  for (int iteration = 0; iteration < sample_num_ && best_path_.empty(); ++iteration)
  {
    const bool active_is_start = (iteration % 2 == 0);
    Tree& active_tree = active_is_start ? start_tree_ : goal_tree_;
    const Tree& opposite_tree = active_is_start ? goal_tree_ : start_tree_;
    const Node& attractive_root = active_is_start ? goal_ : start_;

    Node target = _biasedTarget(opposite_tree);
    Node new_node;
    bool extended = _extendTree(active_tree, target, attractive_root, new_node);

    //Random-sample retry
    if (!extended)
    {
      target = _uniformSample();
      extended = _extendTree(active_tree, target, attractive_root, new_node);
    }

    if (!extended)
      continue;

    expand.push_back(new_node);
    _tryConnect(active_is_start, new_node);
  }

  if (best_path_.empty())
    return false;

  // Phase 2:
  for (int iteration = 0; iteration < informed_iterations_; ++iteration)
  {
    if (informed_threshold_cost_ > 0.0 && c_best_ <= informed_threshold_cost_)
      break;

    const bool active_is_start = (iteration % 2 == 0);
    Tree& active_tree = active_is_start ? start_tree_ : goal_tree_;
    const Node& attractive_root = active_is_start ? goal_ : start_;

    Node target = _informedSample(c_best_);
    Node new_node;
    if (!_extendTree(active_tree, target, attractive_root, new_node))
      continue;

    expand.push_back(new_node);
    _tryConnect(active_is_start, new_node);
  }

  // Sec. 3.6: the remaining key nodes become the sequence of local sub-goals.
  const std::vector<Node> key_nodes = _extractKeyNodes(best_path_);
  if (key_nodes.empty())
    return false;

  _resetSubGoals(key_nodes);


  cached_path_ = _densify(key_nodes);
  cached_expand_ = expand;
  cached_goal_id_ = goal.id();
  has_plan_ = !cached_path_.empty();

  path = cached_path_;
  return has_plan_;
}

bool InformedRRTStar::subGoal(Node& sub_goal) const
{
  if (!has_plan_ || sub_goal_index_ >= key_nodes_.size())
    return false;

  sub_goal = key_nodes_[sub_goal_index_];
  return true;
}

void InformedRRTStar::_resetSubGoals(const std::vector<Node>& key_nodes_goal_to_start)
{
  key_nodes_ = key_nodes_goal_to_start;
  std::reverse(key_nodes_.begin(), key_nodes_.end());  // start -> goal

  // Index 0 is the start itself, so the first sub-goal is the next key node.
  sub_goal_index_ = key_nodes_.size() > 1 ? 1 : 0;
}

void InformedRRTStar::_advanceSubGoal(const Node& start)
{
  if (key_nodes_.size() < 2)
    return;

  const double resolution = std::max(1e-6, static_cast<double>(costmap_->getResolution()));
  const double tolerance_cells = kSubGoalToleranceMetres / resolution;

  // Sec. 3.6
  while (sub_goal_index_ + 1 < key_nodes_.size() &&
         helper::dist(start, key_nodes_[sub_goal_index_]) <= tolerance_cells)
    ++sub_goal_index_;
}

bool InformedRRTStar::_cachedPlanUsable(const Node& start, const Node& goal) const
{
  if (!has_plan_ || cached_path_.size() < 2 || key_nodes_.size() < 2 || goal.id() != cached_goal_id_)
    return false;

  const double resolution = std::max(1e-6, static_cast<double>(costmap_->getResolution()));

  // The robot has to still be near the stored path for the local planner to
  // follow it; otherwise the path is stale no matter how clear it is.
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < cached_path_.size(); ++i)
    nearest_distance = std::min(nearest_distance, helper::dist(cached_path_[i], start));
  if (nearest_distance * resolution > kMaxDeviationMetres)
    return false;

  //Is the current path reachable?
  const std::size_t first = sub_goal_index_ > 0 ? sub_goal_index_ - 1 : 0;
  for (std::size_t i = first; i + 1 < key_nodes_.size(); ++i)
  {
    if (!_collisionFree(key_nodes_[i], key_nodes_[i + 1]))
      return false;
  }

  return true;
}

std::vector<Node> InformedRRTStar::_densify(const std::vector<Node>& polyline) const
{
  if (polyline.size() < 2)
    return polyline;

  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  std::vector<Node> dense;
  dense.push_back(polyline.front());

  for (std::size_t i = 0; i + 1 < polyline.size(); ++i)
  {
    const Node& from = polyline[i];
    const Node& to = polyline[i + 1];
    const int steps = std::max(1, static_cast<int>(std::ceil(helper::dist(from, to))));

    for (int step = 1; step <= steps; ++step)
    {
      const double ratio = static_cast<double>(step) / steps;
      const int x = static_cast<int>(std::round(from.x() + ratio * (to.x() - from.x())));
      const int y = static_cast<int>(std::round(from.y() + ratio * (to.y() - from.y())));
      if (x == dense.back().x() && y == dense.back().y())
        continue;

      dense.push_back(Node(x, y, 0.0, 0.0, x + width * y, dense.back().id()));
    }
  }

  return dense;
}

void InformedRRTStar::_resetTree(Tree& tree, const Node& root)
{
  tree.nodes.clear();
  tree.insertion_order.clear();

  Node root_node = root;
  root_node.set_g(0.0);
  root_node.set_pid(root.id());
  tree.nodes.insert(std::make_pair(root_node.id(), root_node));
  tree.insertion_order.push_back(root_node.id());
  tree.root_id = root_node.id();
  tree.last_id = root_node.id();
}

Node InformedRRTStar::_uniformSample()
{
  std::uniform_int_distribution<int> dist_x(0, static_cast<int>(costmap_->getSizeInCellsX()) - 1);
  std::uniform_int_distribution<int> dist_y(0, static_cast<int>(costmap_->getSizeInCellsY()) - 1);

  const int x = dist_x(rng_);
  const int y = dist_y(rng_);
  return Node(x, y, 0.0, 0.0, grid2Index(x, y), 0);
}

Node InformedRRTStar::_informedSample(double c_best)
{
  const double c_min = helper::dist(start_, goal_);
  if (!std::isfinite(c_best) || c_min <= std::numeric_limits<double>::epsilon())
    return _uniformSample();

  const Eigen::Vector2d centre((start_.x() + goal_.x()) / 2.0, (start_.y() + goal_.y()) / 2.0);
  const double heading = std::atan2(goal_.y() - start_.y(), goal_.x() - start_.x());

  Eigen::Matrix2d rotation;
  rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);

  const double major_radius = c_best / 2.0;
  const double minor_squared = std::max(0.0, c_best * c_best - c_min * c_min);
  const double minor_radius = std::sqrt(minor_squared) / 2.0;

  Eigen::Matrix2d scale = Eigen::Matrix2d::Zero();
  scale(0, 0) = major_radius;
  scale(1, 1) = minor_radius;

  for (int attempt = 0; attempt < 1000; ++attempt)
  {
    const double theta = 2.0 * M_PI * _random01();
    const double radius = std::sqrt(_random01());
    const Eigen::Vector2d unit_ball(radius * std::cos(theta), radius * std::sin(theta));
    const Eigen::Vector2d sample = rotation * scale * unit_ball + centre;

    const int x = static_cast<int>(std::round(sample.x()));
    const int y = static_cast<int>(std::round(sample.y()));
    Node node(x, y, 0.0, 0.0, _isValid(Node(x, y)) ? grid2Index(x, y) : -1, 0);
    if (_isValid(node))
      return node;
  }

  return Node(0, 0, 0.0, 0.0, -1, 0);
}

Node InformedRRTStar::_biasedTarget(const Tree& opposite_tree)
{
  if (opposite_tree.nodes.empty())
    return _uniformSample();

  // Eq. (11)
  if (_random01() <= goal_bias_probability_)
    return opposite_tree.nodes.at(opposite_tree.last_id);

  return _randomOppositeNode(opposite_tree);
}

Node InformedRRTStar::_randomOppositeNode(const Tree& opposite_tree)
{
  std::vector<int> candidates;
  candidates.reserve(opposite_tree.insertion_order.size());
  for (std::size_t i = 0; i < opposite_tree.insertion_order.size(); ++i)
  {
    const int id = opposite_tree.insertion_order[i];
    if (id != opposite_tree.last_id)
      candidates.push_back(id);
  }

  
  if (candidates.empty())
    return _uniformSample();

  std::uniform_int_distribution<std::size_t> dist(0, candidates.size() - 1);
  return opposite_tree.nodes.at(candidates[dist(rng_)]);
}

Node InformedRRTStar::_nearestNode(const Tree& tree, const Node& target) const
{
  Node nearest;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::unordered_map<int, Node>::const_iterator it = tree.nodes.begin(); it != tree.nodes.end(); ++it)
  {
    const double distance = helper::dist(it->second, target);
    if (distance < nearest_distance)
    {
      nearest = it->second;
      nearest_distance = distance;
    }
  }
  return nearest;
}

Node InformedRRTStar::_steer(const Node& nearest, const Node& target, double step) const
{
  const double distance = helper::dist(nearest, target);
  if (distance <= step)
    return Node(target.x(), target.y(), 0.0, 0.0, target.id(), nearest.id());

  const int x = nearest.x() + static_cast<int>(std::round((target.x() - nearest.x()) * step / distance));
  const int y = nearest.y() + static_cast<int>(std::round((target.y() - nearest.y()) * step / distance));
  const int id = x + static_cast<int>(costmap_->getSizeInCellsX()) * y;
  return Node(x, y, 0.0, 0.0, id, nearest.id());
}

bool InformedRRTStar::_extendTree(Tree& tree, const Node& target, const Node& attractive_root, Node& new_node)
{
  if (!_isValid(target))
    return false;

  const Node nearest = _nearestNode(tree, target);
  const double step = _adaptiveStep(target, attractive_root);
  Node candidate = _steer(nearest, target, step);

  if (!_isValid(candidate) || candidate.id() == nearest.id() || tree.nodes.find(candidate.id()) != tree.nodes.end() ||
      _isInObstacle(candidate) || !_collisionFree(nearest, candidate))
    return false;

  if (!_insertAndRewire(tree, candidate, nearest))
    return false;

  new_node = candidate;
  return true;
}

bool InformedRRTStar::_insertAndRewire(Tree& tree, Node& new_node, const Node& nearest)
{
  int best_parent_id = nearest.id();
  double best_cost = nearest.g() + helper::dist(nearest, new_node);

  // RRT*: choose the minimum-cost collision-free parent in the neighborhood.
  for (std::unordered_map<int, Node>::const_iterator it = tree.nodes.begin(); it != tree.nodes.end(); ++it)
  {
    const Node& candidate_parent = it->second;
    const double edge_length = helper::dist(candidate_parent, new_node);
    if (edge_length > search_radius_ || !_collisionFree(candidate_parent, new_node))
      continue;

    const double candidate_cost = candidate_parent.g() + edge_length;
    if (candidate_cost < best_cost)
    {
      best_cost = candidate_cost;
      best_parent_id = candidate_parent.id();
    }
  }

  new_node.set_pid(best_parent_id);
  new_node.set_g(best_cost);
  if (!tree.nodes.insert(std::make_pair(new_node.id(), new_node)).second)
    return false;

  tree.insertion_order.push_back(new_node.id());
  tree.last_id = new_node.id();

  // RRT*: rewire neighbors through the new node when this lowers their cost.
  for (std::size_t i = 0; i < tree.insertion_order.size(); ++i)
  {
    const int neighbor_id = tree.insertion_order[i];
    if (neighbor_id == new_node.id() || neighbor_id == best_parent_id || neighbor_id == tree.root_id)
      continue;

    Node& neighbor = tree.nodes.at(neighbor_id);
    const double edge_length = helper::dist(new_node, neighbor);
    if (edge_length > search_radius_ || new_node.g() + edge_length >= neighbor.g() ||
        _isAncestor(tree, neighbor.id(), new_node.id()) || !_collisionFree(new_node, neighbor))
      continue;

    neighbor.set_pid(new_node.id());
    neighbor.set_g(new_node.g() + edge_length);
    std::unordered_set<int> visited;
    _updateDescendantCosts(tree, neighbor.id(), visited);
  }

  return true;
}

bool InformedRRTStar::_tryConnect(bool active_is_start, const Node& new_node)
{
  const Tree& opposite_tree = active_is_start ? goal_tree_ : start_tree_;
  const Node opposite_node = _nearestNode(opposite_tree, new_node);

  if (helper::dist(new_node, opposite_node) > search_radius_ || !_collisionFree(new_node, opposite_node))
    return false;

  if (active_is_start)
    _considerConnection(new_node.id(), opposite_node.id());
  else
    _considerConnection(opposite_node.id(), new_node.id());
  return true;
}

void InformedRRTStar::_considerConnection(int start_tree_id, int goal_tree_id)
{
  const std::unordered_map<int, Node>::const_iterator start_it = start_tree_.nodes.find(start_tree_id);
  const std::unordered_map<int, Node>::const_iterator goal_it = goal_tree_.nodes.find(goal_tree_id);
  if (start_it == start_tree_.nodes.end() || goal_it == goal_tree_.nodes.end())
    return;

  const double candidate_cost =
      start_it->second.g() + helper::dist(start_it->second, goal_it->second) + goal_it->second.g();
  if (candidate_cost >= c_best_)
    return;

  const std::vector<Node> candidate_path = _buildPath(start_tree_id, goal_tree_id);
  if (candidate_path.empty())
    return;

  c_best_ = candidate_cost;
  best_path_ = candidate_path;
}

std::vector<Node> InformedRRTStar::_traceToRoot(const Tree& tree, int node_id) const
{
  std::vector<Node> trace;
  std::unordered_set<int> visited;
  int current_id = node_id;

  while (visited.insert(current_id).second)
  {
    const std::unordered_map<int, Node>::const_iterator it = tree.nodes.find(current_id);
    if (it == tree.nodes.end())
      return std::vector<Node>();

    trace.push_back(it->second);
    if (current_id == tree.root_id)
      return trace;
    current_id = it->second.pid();
  }

  return std::vector<Node>();
}

std::vector<Node> InformedRRTStar::_buildPath(int start_tree_id, int goal_tree_id) const
{
  // start_trace: connection -> ... -> start
  const std::vector<Node> start_trace = _traceToRoot(start_tree_, start_tree_id);
  // goal_trace: connection -> ... -> goal
  std::vector<Node> goal_trace = _traceToRoot(goal_tree_, goal_tree_id);
  if (start_trace.empty() || goal_trace.empty())
    return std::vector<Node>();

  // The rest of this package expects goal -> ... -> start.
  std::reverse(goal_trace.begin(), goal_trace.end());
  std::vector<Node> result = goal_trace;

  std::size_t start_index = 0;
  if (!result.empty() && result.back().id() == start_trace.front().id())
    start_index = 1;
  result.insert(result.end(), start_trace.begin() + start_index, start_trace.end());
  return result;
}

std::vector<Node> InformedRRTStar::_extractKeyNodes(const std::vector<Node>& raw_path) const
{
  if (raw_path.size() <= 2)
    return raw_path;

  // raw_path follows goal -> start.
  std::vector<Node> key_nodes;
  key_nodes.push_back(raw_path.front());

  std::size_t current = 0;
  while (current + 1 < raw_path.size())
  {
    std::size_t next = raw_path.size() - 1;
    while (next > current + 1 && !_collisionFree(raw_path[current], raw_path[next]))
      --next;

    key_nodes.push_back(raw_path[next]);
    current = next;
  }

  return key_nodes;
}

bool InformedRRTStar::_isAncestor(const Tree& tree, int ancestor_id, int node_id) const
{
  int current_id = node_id;
  for (std::size_t depth = 0; depth <= tree.nodes.size(); ++depth)
  {
    if (current_id == ancestor_id)
      return true;
    if (current_id == tree.root_id)
      return false;

    const std::unordered_map<int, Node>::const_iterator it = tree.nodes.find(current_id);
    if (it == tree.nodes.end())
      return false;
    current_id = it->second.pid();
  }
  return true;
}

void InformedRRTStar::_updateDescendantCosts(Tree& tree, int parent_id, std::unordered_set<int>& visited)
{
  if (!visited.insert(parent_id).second)
    return;

  const Node parent = tree.nodes.at(parent_id);
  for (std::unordered_map<int, Node>::iterator it = tree.nodes.begin(); it != tree.nodes.end(); ++it)
  {
    Node& child = it->second;
    if (child.id() == parent_id || child.pid() != parent_id)
      continue;

    child.set_g(parent.g() + helper::dist(parent, child));
    _updateDescendantCosts(tree, child.id(), visited);
  }
}

bool InformedRRTStar::_isValid(const Node& node) const
{
  return node.x() >= 0 && node.x() < static_cast<int>(costmap_->getSizeInCellsX()) && node.y() >= 0 &&
         node.y() < static_cast<int>(costmap_->getSizeInCellsY());
}

bool InformedRRTStar::_isInObstacle(const Node& node) const
{
  return !_isValid(node) ||
         costmap_->getCharMap()[node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_;
}

bool InformedRRTStar::_collisionFree(const Node& n1, const Node& n2) const
{
  if (!_isValid(n1) || !_isValid(n2) || _isInObstacle(n1) || _isInObstacle(n2))
    return false;

  // Checking
  const int samples = std::max(1, static_cast<int>(std::ceil(helper::dist(n1, n2))));
  for (int i = 0; i <= samples; ++i)
  {
    const double ratio = static_cast<double>(i) / samples;
    const int x = static_cast<int>(std::round(n1.x() + ratio * (n2.x() - n1.x())));
    const int y = static_cast<int>(std::round(n1.y() + ratio * (n2.y() - n1.y())));
    const int id = x + static_cast<int>(costmap_->getSizeInCellsX()) * y;
    Node point(x, y, 0.0, 0.0, id, 0);
    if (_isInObstacle(point))
      return false;
  }
  return true;
}

void InformedRRTStar::_buildClearanceMap()
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  clearance_map_.assign(width * height, std::numeric_limits<double>::infinity());

  typedef std::pair<double, int> QueueEntry;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry> > queue;

  for (int id = 0; id < width * height; ++id)
  {
    if (costmap_->getCharMap()[id] >= costmap_2d::LETHAL_OBSTACLE * factor_)
    {
      clearance_map_[id] = 0.0;
      queue.push(std::make_pair(0.0, id));
    }
  }

  const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
  const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
  const double diagonal = std::sqrt(2.0);

  while (!queue.empty())
  {
    const QueueEntry current = queue.top();
    queue.pop();
    if (current.first > clearance_map_[current.second])
      continue;

    const int x = current.second % width;
    const int y = current.second / width;
    for (int direction = 0; direction < 8; ++direction)
    {
      const int nx = x + dx[direction];
      const int ny = y + dy[direction];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height)
        continue;

      const int next_id = nx + width * ny;
      const double edge_cost = direction < 4 ? 1.0 : diagonal;
      const double next_distance = current.first + edge_cost;
      if (next_distance < clearance_map_[next_id])
      {
        clearance_map_[next_id] = next_distance;
        queue.push(std::make_pair(next_distance, next_id));
      }
    }
  }
}

double InformedRRTStar::_obstacleClearance(const Node& node) const
{
  if (!_isValid(node) || node.id() < 0 || node.id() >= static_cast<int>(clearance_map_.size()))
    return 0.0;
  return clearance_map_[node.id()] * costmap_->getResolution();
}

double InformedRRTStar::_adaptiveStep(const Node& sample, const Node& attractive_root) const
{
  const double resolution = std::max(1e-6, static_cast<double>(costmap_->getResolution()));
  const double obstacle_distance = std::max(resolution, _obstacleClearance(sample));
  const double goal_distance = helper::dist(sample, attractive_root) * resolution;

  // Paper Eq. (12): attractive potential at the sampled point.
  const double att = 0.5 * attractive_gain_ * goal_distance * goal_distance;


  // Paper Eq. (13): repulsive potential within the obstacle influence range.
  double rep = 0.0;
  if (obstacle_distance <= obstacle_influence_distance_ && repulsive_gain_ > 0.0)
  {
    const double inverse_difference = 1.0 / obstacle_distance - 1.0 / obstacle_influence_distance_;
    rep = 0.5 * repulsive_gain_ * inverse_difference * inverse_difference;
  }

  // Paper Eq. (14). The lower/upper clamp only prevents a non-positive log
  // argument and keeps the extension inside configured numerical bounds.
  double step_metres = 0.0;
  if (obstacle_distance <= danger_distance_)
  {
    step_metres = std::exp(-rep);
  }
  else if (obstacle_distance <= near_distance_)
  {
    // Eq. (14)
    const double eta = std::max(repulsive_gain_, 1e-9);
    const double inverse_distance =
        std::sqrt(std::max(0.0, 2.0 * rep / eta)) + 1.0 / obstacle_influence_distance_;
    step_metres = std::log(std::max(att * inverse_distance, 1.0));
  }
  else
  {
    step_metres = std::log(std::max(att, 1.0));
  }

  double step_cells = step_metres / resolution;
  if (!std::isfinite(step_cells))
    step_cells = min_step_;
  return std::max(min_step_, std::min(max_step_, step_cells));
}

double InformedRRTStar::_random01()
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(rng_);
}
}  // namespace global_planner
