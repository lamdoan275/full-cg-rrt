/**
 * *********************************************************
 *
 * @file: cg_rrt_paper.cpp
 * @brief: Confidence-Guided RRT (CG-RRT) -- reimplementation of the manuscript
 *         "Confidence-Guided RRT for Mobile Service Robot Navigation in
 *         Cluttered Indoor Environments" (JIST-D-26-00274).
 *
 * See cg_rrt_paper.h for the algorithm-to-method mapping and for the complete
 * list of deviations from the pseudocode. Every deviation is marked "[impl]"
 * at the line that introduces it.
 *
 * ********************************************************
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <ros/ros.h>

#include "cg_rrt_paper.h"

namespace global_planner
{
namespace
{
// Get_Confidence() scans D = 8 directions at 45 degree intervals (Sec. 2.2).
constexpr int CONFIDENCE_DIRECTIONS = 8;
constexpr double CONFIDENCE_STEP_DEG = 360.0 / CONFIDENCE_DIRECTIONS;

// Algorithm 2 generates N = 6 candidates per layer at 60 degree intervals.
constexpr double CANDIDATE_STEP_DEG = 60.0;

/**
 * @brief Milliseconds elapsed since a steady-clock stamp
 * @param since  the earlier stamp
 * @return the elapsed time in milliseconds
 */
double elapsedMs(const std::chrono::steady_clock::time_point& since)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}
}  // namespace

/**
 * @brief  Constructor
 * @param costmap      the environment for path planning
 * @param n_max        Table 1 nmax, max points in the search process
 * @param d_max        Table 1 dmax, max distance between consecutive nodes [m]
 * @param eps          Table 1 epsilon, acceptable distance to the goal [m]
 * @param k            Table 1 k, nodes expanded in the K-nearest method
 * @param n_lim        Table 1 nlim, max nodes in one Modified-RRT tree
 * @param c_max        Table 1 cmax, max confidence distance [m]
 * @param c_min        min confidence a candidate must keep [m] (not in Table 1)
 * @param layers       Section 3.1 L, number of sub-goal sampling layers
 * @param max_cuts     max cuts chained into one phase route (not in the paper)
 * @param max_failures consecutive phase failures before a sub-goal is dropped
 */
CGRRTPaper::CGRRTPaper(costmap_2d::Costmap2D* costmap, int n_max, double d_max, double eps, int k, int n_lim,
                       double c_max, double c_min, int layers, int max_cuts, int max_failures)
  // Table 1 is written in metres while every node coordinate here is a costmap
  // cell, so dmax is converted before it reaches the RRT base class.
  : RRTAStar(costmap, n_max, d_max / costmap->getResolution(), k)
  , eps_(eps / costmap->getResolution())
  , n_lim_(n_lim)
  , c_max_(c_max / costmap->getResolution())
  , c_min_(c_min / costmap->getResolution())
  , layers_(layers)
  , max_cut_segments_(max_cuts)
  , max_phase_failures_(max_failures)
  , has_sub_goal_(false)
  , sub_goal_failures_(0)
  , has_spent_sub_goal_(false)
  , has_task_goal_(false)
  , phase_id_(0)
{
  // Algorithm 3 line 7 is a plain Rand_State(); the RRT base class otherwise
  // returns qgoal for 5% of its samples, which is a sampling bias the
  // manuscript does not describe.
  opti_sample_p_ = 0.0;

  ROS_INFO("CG-RRT (paper): nmax=%d dmax=%.2fm(%.1f cells) eps=%.2fm(%.1f cells) k=%d nlim=%d cmax=%.2fm cmin=%.2fm "
           "L=%d max_cuts=%d max_phase_failures=%d",
           n_max, d_max, max_dist_, eps, eps_, k, n_lim_, c_max, c_min, layers_, max_cut_segments_,
           max_phase_failures_);
}

/**
 * @brief Algorithm 1, one CG-RRT planning cycle
 * @param start   robot location, i.e. qinit / qstart
 * @param goal    qgoal
 * @param path    resulting path, ordered goal -> start
 * @param expand  the nodes searched during the process
 * @return true if a path was produced, else false
 */
bool CGRRTPaper::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // line 4: qinit <- GetRobot_Location(). move_base passes the current robot
  // pose into every makePlan() call, so `start` is qinit as well as qstart.
  const Node q_init = start;

  // per-cycle instrumentation; phase_failures accumulates over the whole task
  metrics_.cut_segments = 0;
  metrics_.expanded_nodes = 0;
  metrics_.confidence_ms = 0.0;
  metrics_.modified_rrt_ms = 0.0;

  // [impl] a new qgoal starts a new invocation of the procedure, which owns all
  // of the state below.
  if (!has_task_goal_ || task_goal_.id() != goal.id())
  {
    task_goal_ = goal;
    has_task_goal_ = true;
    has_sub_goal_ = false;
    has_spent_sub_goal_ = false;
    sub_goal_failures_ = 0;
    metrics_.phase_failures = 0;
    ++phase_id_;
  }

  // [impl] line 7: `while qinit != qsub_goal` runs across replanning cycles.
  // The sub-goal is released only once the robot actually stands on it, so the
  // path handed to the local planner keeps terminating at qsub_goal until then.
  if (has_sub_goal_)
  {
    if (helper::dist(q_init, sub_goal_) <= eps_)
      _releaseSubGoal("reached, planning to qgoal");
    else if (!_isFreeNode(sub_goal_))
      // [impl] the costmap is updated by the sensors between cycles; a sub-goal
      // that has become lethal cannot be driven to.
      _releaseSubGoal("no longer free");
  }

  // lines 5-6: if the direct connection conflicts with an obstacle, generate a
  // sub-goal with the confidence-guided mechanism. `!has_spent_sub_goal_` is
  // the `if` of line 5 being evaluated once per procedure call, see the header:
  // Algorithm 2 is deterministic in the map, so re-running it after its sub-goal
  // has been worked off only sends the robot back to the same place.
  if (!has_sub_goal_ && !has_spent_sub_goal_ && checkConflict(q_init, goal))
  {
    Node q_sub_goal;

    const auto t_confidence = std::chrono::steady_clock::now();
    const bool generated = confidenceGoal(goal, q_sub_goal);
    metrics_.confidence_ms = elapsedMs(t_confidence);

    if (generated && _isFreeNode(q_sub_goal) && helper::dist(q_init, q_sub_goal) > eps_)
    {
      sub_goal_ = q_sub_goal;
      has_sub_goal_ = true;
      sub_goal_failures_ = 0;
      ++phase_id_;
      ROS_INFO("CG-RRT (paper): qsub_goal = (%d, %d), %.1f cells from qgoal.", sub_goal_.x(), sub_goal_.y(),
               helper::dist(sub_goal_, goal));
    }
  }

  // line 8 while a sub-goal is active, line 12 once it has been reached. The
  // cuts Modified-RRT produces on the way are chained into one verified route.
  const bool escaping = has_sub_goal_;
  const Node target = escaping ? sub_goal_ : goal;
  const bool found = buildPhasePath(start, target, path, expand);

  if (!found)
    ++metrics_.phase_failures;

  if (has_sub_goal_)
  {
    if (found)
      sub_goal_failures_ = 0;
    else if (++sub_goal_failures_ >= max_phase_failures_)
      _releaseSubGoal("unreachable");
  }

  ROS_INFO("CG-RRT (paper) [%s] %s: cuts=%d expanded=%d route=%lu t_confidence=%.2fms t_modified_rrt=%.2fms "
           "phase_failures=%d",
           escaping ? "qsub_goal" : "qgoal", found ? "route" : "FAILED", metrics_.cut_segments,
           metrics_.expanded_nodes, static_cast<unsigned long>(path.size()), metrics_.confidence_ms,
           metrics_.modified_rrt_ms, metrics_.phase_failures);

  return found;
}

/**
 * @brief Chain the cuts of one phase into a single verified route
 * @param start   where the route starts, i.e. the robot
 * @param target  qsub_goal while escaping, qgoal afterwards
 * @param path    resulting route, ordered target -> start, empty on failure
 * @param expand  the nodes expanded by every Modified-RRT call made here
 * @return true only if the route reaches target with every edge clear
 */
bool CGRRTPaper::buildPhasePath(const Node& start, const Node& target, std::vector<Node>& path,
                                std::vector<Node>& expand)
{
  path.clear();
  expand.clear();

  // built forwards, start -> target, and reversed once it is complete
  std::vector<Node> route;
  route.push_back(start);

  // [impl] loop guard: two cuts ending on the same cell would keep re-running
  // the same expansion from the same place
  std::unordered_set<int> endpoints;
  endpoints.insert(start.id());

  Node current = start;

  for (int segment = 0; segment < max_cut_segments_; ++segment)
  {
    std::vector<Node> cut_path, cut_expand;

    const auto t_rrt = std::chrono::steady_clock::now();
    const bool found = modifiedRRT(current, target, cut_path, cut_expand);
    metrics_.modified_rrt_ms += elapsedMs(t_rrt);

    ++metrics_.cut_segments;
    metrics_.expanded_nodes += static_cast<int>(cut_expand.size());
    expand.insert(expand.end(), cut_expand.begin(), cut_expand.end());

    if (!found || cut_path.empty())
    {
      ROS_WARN("CG-RRT (paper): Modified-RRT produced no usable cut at segment %d.", segment);
      path.clear();
      return false;
    }

    // Modified-RRT returns target -> start; walk the cut forwards instead
    std::reverse(cut_path.begin(), cut_path.end());

    for (std::size_t i = 1; i < cut_path.size(); ++i)
    {
      // _convertClosedListToPath rebuilds its nodes from x and y alone, so the
      // ids have to be restored before a node can be reused or compared
      Node& next = cut_path[i];
      next.set_id(grid2Index(next.x(), next.y()));

      // every edge of the delivered route is verified: the local planner is
      // never handed a segment that crosses an obstacle
      if (_collisionObstacle(cut_path[i - 1], next))
      {
        ROS_WARN("CG-RRT (paper): rejecting a blocked edge (%d, %d) -> (%d, %d) at segment %d.", cut_path[i - 1].x(),
                 cut_path[i - 1].y(), next.x(), next.y(), segment);
        path.clear();
        return false;
      }

      route.push_back(next);
    }

    const Node endpoint = route.back();

    // the phase target has been reached, the route is complete
    if (endpoint == target)
    {
      path.assign(route.rbegin(), route.rend());
      return true;
    }

    // a cut that ends where an earlier one did, the degenerate case being one
    // that made no progress at all, would replay the same expansion forever
    if (!endpoints.insert(endpoint.id()).second)
    {
      ROS_WARN("CG-RRT (paper): cut %d returned to (%d, %d), abandoning this phase route.", segment, endpoint.x(),
               endpoint.y());
      path.clear();
      return false;
    }

    // the cut node becomes the root of the next Modified-RRT tree
    current = endpoint;
    current.set_g(0.0);
    current.set_pid(0);
  }

  ROS_WARN("CG-RRT (paper): target (%d, %d) not reached within %d cuts.", target.x(), target.y(), max_cut_segments_);
  path.clear();
  return false;
}

/**
 * @brief Leave the while loop of Algorithm 1 and remember the sub-goal as
 *        spent, so it is never latched a second time
 * @param reason  why the sub-goal is being released, for the log line
 */
void CGRRTPaper::_releaseSubGoal(const char* reason)
{
  ROS_INFO("CG-RRT (paper): releasing qsub_goal (%d, %d): %s.", sub_goal_.x(), sub_goal_.y(), reason);

  has_spent_sub_goal_ = true;
  has_sub_goal_ = false;
  sub_goal_failures_ = 0;

  // the qgoal phase starts here; anything planned for the sub-goal phase, the
  // wrapper's stored plan included, no longer describes where the robot is going
  ++phase_id_;
}

/**
 * @brief Algorithm 3, Modified-RRT: RRT-A* expansion under a node limit
 * @param start   qstart
 * @param goal    the current target, either qsub_goal or qgoal
 * @param path    resulting path, ordered target -> start
 * @param expand  the nodes searched during the process
 * @return true if a full or cut path was produced, else false
 */
bool CGRRTPaper::modifiedRRT(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  path.clear();
  expand.clear();
  sample_list_.clear();

  // lines 2-3: TR.V <- {qstart}, TR.E <- {}, p <- {}, iteration <- 0, n <- 1
  start_ = start;
  goal_ = goal;
  sample_list_.insert(std::make_pair(start.id(), start));
  expand.push_back(start);

  int iteration = 0;
  int n = 1;

  // line 4
  while (iteration < sample_num_)
  {
    // [impl] the pseudocode advances `iteration` only on a successful
    // expansion (line 26). Counting every pass instead bounds the run at nmax
    // passes even when samples keep landing in obstacles.
    ++iteration;

    // lines 5-12: draw k samples and keep the one whose nearest tree node has
    // the smallest f = g + h, Eq. (4).
    double best_f = std::numeric_limits<double>::infinity();
    Node best_near, best_rand;
    bool has_candidate = false;

    for (int i = 0; i < k_; ++i)
    {
      // line 7: qrand <- Rand_State()
      const Node q_rand = _generateRandomNode();
      if (!_isFreeNode(q_rand))
        continue;
      if (sample_list_.find(q_rand.id()) != sample_list_.end())
        continue;

      // line 8: qnear <- Nearest(TR, qrand)
      Node q_near = _getNearestNode(sample_list_, q_rand);

      // line 9: f <- g(qnear) + h(qnear)
      const double f = q_near.g() + helper::dist(q_near, goal_);
      if (f < best_f)
      {
        best_f = f;
        best_near = q_near;
        best_rand = q_rand;
        has_candidate = true;
      }
    }

    if (!has_candidate)
      continue;

    // line 13: qnew <- Steer(qnear, qrand, dmax)
    Node q_new = _getNewNode(best_near, best_rand);

    // [impl] Steer() is assumed collision-free in the pseudocode. Here it
    // reports a blocked edge with id == -1, and such a sample is dropped
    // without touching the tree or the node counter.
    if (q_new.id() == -1)
      continue;

    // lines 24-25: TR.V <- TR.V + {qnew}, TR.E <- TR.E + {(qnear, qnew)}.
    // Done before the two tests below because both back-track through qnew.
    if (!sample_list_.insert(std::make_pair(q_new.id(), q_new)).second)
      continue;
    expand.push_back(q_new);

    // line 14: n <- n + 1
    ++n;

    // lines 15-18: the target is within epsilon, return the complete path
    if (_checkGoalEps(q_new))
    {
      path = _convertClosedListToPath(sample_list_, start, goal_);
      return !path.empty();
    }

    // lines 19-23: node limitation. Stop this expansion phase and cut the tree
    // at its most promising node, the one minimising f = g + h.
    if (n >= n_lim_)
    {
      Node q_jump;
      bool has_jump = false;
      double min_f = std::numeric_limits<double>::infinity();

      for (const auto& entry : sample_list_)
      {
        const Node& node = entry.second;

        // [impl] the root would back-track to a zero-length path
        if (node.id() == start.id())
          continue;

        const double f = node.g() + helper::dist(node, goal_);  // Eq. (4)
        if (f < min_f)
        {
          min_f = f;
          q_jump = node;
          has_jump = true;
        }
      }

      if (!has_jump)
        return false;

      path = _convertClosedListToPath(sample_list_, start, q_jump);
      return !path.empty();
    }
  }

  // line 28: the budget ran out without a path
  return false;
}

/**
 * @brief Algorithm 2, Confidence_Goal: the L-layer confidence search
 * @param q_goal      qgoal, the origin of the first sampling circle
 * @param q_sub_goal  filled with the surviving candidate farthest from qgoal
 * @return true if any candidate survived every layer, else false
 */
bool CGRRTPaper::confidenceGoal(const Node& q_goal, Node& q_sub_goal)
{
  // line 2: d0 <- Get_Confidence(qgoal)
  const double d0 = _getConfidence(q_goal);
  if (d0 <= c_min_)
  {
    ROS_WARN("CG-RRT (paper): confidence at qgoal is %.1f cells, below cmin.", d0);
    return false;
  }

  // line 3: dist <- -1
  double best_dist = -1.0;

  // lines 4-36: L nested layers of N = 6 candidates each
  _confidenceLayer(q_goal, d0, 1, q_goal, q_sub_goal, best_dist);

  if (best_dist < 0.0)
    return false;

  q_sub_goal.set_id(grid2Index(q_sub_goal.x(), q_sub_goal.y()));
  return true;
}

/**
 * @brief One layer of Algorithm 2: N = 6 candidates at 60 degree intervals
 * @param q_prev     node the sampling circle is centred on
 * @param d_prev     radius of that circle, i.e. the previous confidence
 * @param layer      1-based index of this layer
 * @param q_goal     qgoal, used for the distance being maximised
 * @param q_sub_goal best candidate so far
 * @param best_dist  ||qgoal - q_sub_goal||, -1 while nothing survived
 */
void CGRRTPaper::_confidenceLayer(const Node& q_prev, double d_prev, int layer, const Node& q_goal, Node& q_sub_goal,
                                  double& best_dist)
{
  for (double theta = 0.0; theta < 360.0; theta += CANDIDATE_STEP_DEG)
  {
    // qi <- qi-1 + di-1 * direction(theta)
    const Node q = _getState(q_prev, d_prev, theta);

    // di <- Get_Confidence(qi)
    const double d = _getConfidence(q);

    // if di <= cmin or CollisionObstacle(qi, qi-1) then continue
    if (d <= c_min_)
      continue;
    if (_collisionObstacle(q, q_prev))
      continue;

    if (layer >= layers_)
    {
      // lines 28-32: keep the surviving candidate farthest from qgoal
      const double dist = helper::dist(q_goal, q);
      if (best_dist <= dist)
      {
        q_sub_goal = q;
        best_dist = dist;
      }
    }
    else
    {
      _confidenceLayer(q, d, layer + 1, q_goal, q_sub_goal, best_dist);
    }
  }
}

/**
 * @brief Check_Conflict(): does the straight connection qinit -> qgoal hit an
 *        obstacle?
 * @param q_init  robot location
 * @param q_goal  final goal
 * @return true if the direct connection is blocked
 */
bool CGRRTPaper::checkConflict(const Node& q_init, const Node& q_goal)
{
  return _collisionObstacle(q_init, q_goal);
}

/**
 * @brief The sub-goal currently latched by Algorithm 1
 * @param sub_goal  filled with qsub_goal when one is active
 * @return false while the planner is heading straight for qgoal
 */
bool CGRRTPaper::subGoal(Node& sub_goal) const
{
  if (!has_sub_goal_)
    return false;
  sub_goal = sub_goal_;
  return true;
}

/**
 * @brief Get_Confidence(): radial scan in D = 8 directions at 45 degree
 *        intervals, returning the shortest distance to an obstacle
 * @param q  the node to scan around
 * @return the confidence distance in cells, clamped to cmax, 0 if q is not free
 */
double CGRRTPaper::_getConfidence(const Node& q)
{
  if (!_isFreeNode(q))
    return 0.0;

  double confidence = c_max_;

  for (int i = 0; i < CONFIDENCE_DIRECTIONS; ++i)
  {
    const double theta = i * CONFIDENCE_STEP_DEG * M_PI / 180.0;
    const double dx = std::cos(theta);
    const double dy = std::sin(theta);

    // march outwards one cell at a time until the ray leaves the free space
    double r = 0.0;
    while (r + 1.0 <= c_max_)
    {
      const double next = r + 1.0;
      if (!_isFreeCell(static_cast<int>(std::lround(q.x() + next * dx)),
                       static_cast<int>(std::lround(q.y() + next * dy))))
        break;
      r = next;
    }

    // the smallest distance among the scanned directions is the confidence
    if (r < confidence)
      confidence = r;
  }

  return confidence;
}

/**
 * @brief q + d * direction(theta), rounded onto the costmap grid
 * @param q          base node
 * @param confidence step length in cells
 * @param theta_deg  direction in degrees
 * @return the sampled node
 */
Node CGRRTPaper::_getState(const Node& q, double confidence, double theta_deg)
{
  const double theta = theta_deg * M_PI / 180.0;

  Node state;
  state.set_x(static_cast<int>(std::lround(q.x() + confidence * std::cos(theta))));
  state.set_y(static_cast<int>(std::lround(q.y() + confidence * std::sin(theta))));

  return state;
}

/**
 * @brief CollisionObstacle(): is the straight segment n1 -> n2 blocked?
 * @param n1  first node
 * @param n2  second node
 * @return true if any sampled cell is lethal or off the map
 */
bool CGRRTPaper::_collisionObstacle(const Node& n1, const Node& n2)
{
  const double dx = static_cast<double>(n2.x() - n1.x());
  const double dy = static_cast<double>(n2.y() - n1.y());

  // two samples per cell is enough never to step over a one-cell wall
  const int steps = static_cast<int>(2.0 * std::max(std::fabs(dx), std::fabs(dy))) + 1;

  for (int i = 0; i <= steps; ++i)
  {
    const double t = static_cast<double>(i) / steps;
    if (!_isFreeCell(static_cast<int>(std::lround(n1.x() + dx * t)), static_cast<int>(std::lround(n1.y() + dy * t))))
      return true;
  }

  return false;
}

/**
 * @brief Is this cell on the map and below the lethal threshold?
 * @param x  cell x
 * @param y  cell y
 * @return true if the cell is free
 */
bool CGRRTPaper::_isFreeCell(int x, int y) const
{
  if (x < 0 || y < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
      y >= static_cast<int>(costmap_->getSizeInCellsY()))
    return false;

  return costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) <
         costmap_2d::LETHAL_OBSTACLE * factor_;
}

/**
 * @brief Is this node on the map and below the lethal threshold?
 * @param n  the node
 * @return true if the node's cell is free
 */
bool CGRRTPaper::_isFreeNode(const Node& n) const
{
  return _isFreeCell(n.x(), n.y());
}

/**
 * @brief Algorithm 3 line 15: has qnew come within epsilon of the target?
 * @param new_node  the node just added to the tree
 * @return true if the target was reached, the target is then in sample_list_
 */
bool CGRRTPaper::_checkGoalEps(const Node& new_node)
{
  const double dist = helper::dist(new_node, goal_);
  if (dist > eps_)
    return false;

  // [impl] the last hop has to be drivable; the pseudocode leaves it implicit
  if (_collisionObstacle(new_node, goal_))
    return false;

  const Node target(goal_.x(), goal_.y(), dist + new_node.g(), 0, grid2Index(goal_.x(), goal_.y()), new_node.id());
  sample_list_.insert(std::make_pair(target.id(), target));

  return true;
}
}  // namespace global_planner
