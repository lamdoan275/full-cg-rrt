/**
 * *********************************************************
 *
 * @file: cg_rrt_paper.h
 * @brief: Confidence-Guided RRT (CG-RRT) -- reimplementation of the manuscript
 *         "Confidence-Guided RRT for Mobile Service Robot Navigation in
 *         Cluttered Indoor Environments" (JIST-D-26-00274).
 *
 * This planner is a separate, paper-faithful implementation. It shares no code
 * path with `rrt_cut`, which stays the untouched "GitHub RRT-Cut" baseline.
 *
 * Mapping from the manuscript to this class:
 *   Algorithm 1  CG-RRT                -> plan()
 *   Algorithm 2  Confidence_Goal       -> confidenceGoal() + _confidenceLayer()
 *   Algorithm 3  Modified-RRT          -> modifiedRRT()
 *   Get_Confidence()                   -> _getConfidence()
 *   CollisionObstacle()                -> _collisionObstacle()
 *   Check_Conflict()                   -> checkConflict()
 *   GetRobot_Location()                -> the `start` argument of plan()
 *
 * Table 1 parameters are taken in METRES and converted to costmap cells in the
 * constructor. The baseline planners treat `sample_max_d` as cells, so at the
 * 0.05 m/cell resolution used here their "5.0" is 0.25 m, not the 5 m of
 * Table 1; this class avoids that mismatch by doing the conversion explicitly.
 *
 * Algorithms 2 and 3 are untouched. What sits on top of them is a ROS execution
 * adapter: Algorithm 1 lets the robot drive to each cut node and replans from
 * there, which move_base cannot express, because the local planner is handed
 * one plan per cycle and a cut node is not where the robot is going. So
 * buildPhasePath() stitches the successive cuts of one phase into a single
 * verified route before it leaves the planner. Nothing is appended to a cut
 * path afterwards, and the local planner only ever sees a complete route whose
 * every edge has been collision checked -- the same contract RRT, RRT* and
 * RRT-Connect give it, which is what keeps a shared DWA baseline fair.
 *
 * Deviations from the pseudocode, all of them forced by the ROS integration and
 * each marked "[impl]" at its site:
 *   1. The `while qinit != qsub_goal` loop of Algorithm 1 is spread over
 *      move_base replanning cycles. The sub-goal is latched in the object and
 *      released only once the robot stands within eps of it.
 *   2. Steer() may return a node whose edge crosses an obstacle; such a sample
 *      is discarded instead of being added to the tree.
 *   3. `iteration` is incremented once per while-pass so a run can never exceed
 *      nmax passes.
 *   4. argmin(q.f) skips the tree root, which would back-track to a zero-length
 *      path.
 *   5. cmin is not given in Table 1 and is exposed as a parameter.
 *   6. The RRT base class biases 5% of its samples straight at the goal. That
 *      bias is switched off here, because Algorithm 3 line 7 is a plain
 *      Rand_State(). The other planners in this package keep it.
 *   7. The cuts of one phase are chained inside a single planning cycle instead
 *      of being driven one at a time, see buildPhasePath().
 *
 * ********************************************************
 */
#ifndef CG_RRT_PAPER_H
#define CG_RRT_PAPER_H

#include <unordered_set>

#include "rrt_astar.h"

namespace global_planner
{
/**
 * @brief Confidence-Guided RRT, Algorithms 1-3 of JIST-D-26-00274.
 *
 * RRTAStar is inherited only for its Nearest() (`_getNearestNode`) and Steer()
 * (`_getNewNode`) helpers; RRTAStar::plan() is never called.
 */
class CGRRTPaper : public RRTAStar
{
public:
  /**
   * @brief Per-cycle instrumentation, logged by plan() and readable from the
   *        ROS wrapper. Everything but phase_failures is reset each cycle.
   */
  struct Metrics
  {
    int cut_segments = 0;          // Modified-RRT calls stitched into this cycle
    int expanded_nodes = 0;        // tree nodes over every call, roots included
    double confidence_ms = 0.0;    // Algorithm 2 runtime
    double modified_rrt_ms = 0.0;  // summed Algorithm 3 runtime
    int phase_failures = 0;        // failed phases since the task started
  };

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
  CGRRTPaper(costmap_2d::Costmap2D* costmap, int n_max, double d_max, double eps, int k, int n_lim, double c_max,
             double c_min, int layers, int max_cuts, int max_failures);

  /**
   * @brief Algorithm 1, one CG-RRT planning cycle
   * @param start   robot location, i.e. qinit / qstart
   * @param goal    qgoal
   * @param path    resulting path, ordered goal -> start
   * @param expand  the nodes searched during the process
   * @return true if a path was produced, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * @brief Chain the cuts of one phase into a single verified route
   *
   * Modified-RRT hands back a cut path whenever the node limit bites, so one
   * call rarely reaches the phase target. Algorithm 1 answers that by driving
   * the robot to the cut node and replanning from there; a move_base global
   * planner cannot, so the same walk is performed here instead: plan, take the
   * cut node, append its verified edges, restart from it, repeat. The route is
   * returned only once it genuinely ends on the target.
   *
   * @param start   where the route starts, i.e. the robot
   * @param target  qsub_goal while escaping, qgoal afterwards
   * @param path    resulting route, ordered target -> start, empty on failure
   * @param expand  the nodes expanded by every Modified-RRT call made here
   * @return true only if the route reaches target with every edge clear
   */
  bool buildPhasePath(const Node& start, const Node& target, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * @brief Algorithm 3, Modified-RRT: RRT-A* expansion under a node limit
   * @param start   qstart
   * @param goal    the current target, either qsub_goal or qgoal
   * @param path    resulting path, ordered target -> start
   * @param expand  the nodes searched during the process
   * @return true if a full or cut path was produced, else false
   */
  bool modifiedRRT(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * @brief Algorithm 2, Confidence_Goal: the L-layer confidence search
   * @param q_goal      qgoal, the origin of the first sampling circle
   * @param q_sub_goal  filled with the surviving candidate farthest from qgoal
   * @return true if any candidate survived every layer, else false
   */
  bool confidenceGoal(const Node& q_goal, Node& q_sub_goal);

  /**
   * @brief Check_Conflict(): does the straight connection qinit -> qgoal hit an
   *        obstacle?
   * @param q_init  robot location
   * @param q_goal  final goal
   * @return true if the direct connection is blocked
   */
  bool checkConflict(const Node& q_init, const Node& q_goal);

  /**
   * @brief The sub-goal currently latched by Algorithm 1
   * @param sub_goal  filled with qsub_goal when one is active
   * @return false while the planner is heading straight for qgoal
   */
  bool subGoal(Node& sub_goal) const;

  /**
   * @brief Identifier of the phase being planned, bumped on every new task and
   *        on every sub-goal latch or release
   *
   * A plan built for one phase says nothing about the next one, so the wrapper
   * uses this to refuse to fall back on a stale plan across a phase change.
   * @return the current phase id
   */
  unsigned int phaseId() const
  {
    return phase_id_;
  }

  /**
   * @brief Instrumentation of the last plan() call
   * @return the metrics, see Metrics
   */
  const Metrics& metrics() const
  {
    return metrics_;
  }

protected:
  /**
   * @brief Get_Confidence(): radial scan in D = 8 directions at 45 degree
   *        intervals, returning the shortest distance to an obstacle
   * @param q  the node to scan around
   * @return the confidence distance in cells, clamped to cmax, 0 if q is not free
   */
  double _getConfidence(const Node& q);

  /**
   * @brief One layer of Algorithm 2: N = 6 candidates at 60 degree intervals
   * @param q_prev     node the sampling circle is centred on
   * @param d_prev     radius of that circle, i.e. the previous confidence
   * @param layer      1-based index of this layer
   * @param q_goal     qgoal, used for the distance being maximised
   * @param q_sub_goal best candidate so far
   * @param best_dist  ||qgoal - q_sub_goal||, -1 while nothing survived
   */
  void _confidenceLayer(const Node& q_prev, double d_prev, int layer, const Node& q_goal, Node& q_sub_goal,
                        double& best_dist);

  /**
   * @brief q + d * direction(theta), rounded onto the costmap grid
   * @param q          base node
   * @param confidence step length in cells
   * @param theta_deg  direction in degrees
   * @return the sampled node
   */
  Node _getState(const Node& q, double confidence, double theta_deg);

  /**
   * @brief CollisionObstacle(): is the straight segment n1 -> n2 blocked?
   * @param n1  first node
   * @param n2  second node
   * @return true if any sampled cell is lethal or off the map
   */
  bool _collisionObstacle(const Node& n1, const Node& n2);

  /**
   * @brief Is this cell on the map and below the lethal threshold?
   * @param x  cell x
   * @param y  cell y
   * @return true if the cell is free
   */
  bool _isFreeCell(int x, int y) const;

  /**
   * @brief Is this node on the map and below the lethal threshold?
   * @param n  the node
   * @return true if the node's cell is free
   */
  bool _isFreeNode(const Node& n) const;

  /**
   * @brief Algorithm 3 line 15: has qnew come within epsilon of the target?
   * @param new_node  the node just added to the tree
   * @return true if the target was reached, the target is then in sample_list_
   */
  bool _checkGoalEps(const Node& new_node);

  /**
   * @brief Leave the while loop of Algorithm 1 and remember the sub-goal as
   *        spent, so it is never latched a second time
   * @param reason  why the sub-goal is being released, for the log line
   */
  void _releaseSubGoal(const char* reason);

protected:
  // Table 1, in costmap cells unless stated otherwise
  double eps_;    // epsilon, acceptable distance to the goal
  int n_lim_;     // nlim, max nodes in one Modified-RRT tree
  double c_max_;  // cmax, max confidence distance
  double c_min_;  // cmin, min confidence a candidate must keep
  int layers_;    // L, number of sub-goal sampling layers

  // [impl] execution-adapter limits, no counterpart in the manuscript
  int max_cut_segments_;   // cuts chained into one phase route
  int max_phase_failures_; // consecutive phase failures before dropping a sub-goal

  // [impl] Algorithm 1 state carried across move_base replanning cycles
  Node sub_goal_;          // qsub_goal
  bool has_sub_goal_;      // true while the while-loop of Algorithm 1 is running
  int sub_goal_failures_;  // consecutive Modified-RRT failures toward qsub_goal

  // [impl] Algorithm 1 is one procedure call per navigation task, so its
  // `if Check_Conflict()` branch generates a sub-goal exactly once. move_base
  // calls plan() once per replanning cycle instead, and Algorithm 2 is
  // deterministic in the map, so a spent sub-goal has to be remembered: without
  // this the robot would be sent straight back to it the moment it moved eps
  // away while the goal line was still blocked. Once the sub-goal is spent the
  // task runs on Modified-RRT alone, which is line 12 of Algorithm 1.
  bool has_spent_sub_goal_;  // true once a sub-goal has been released
  Node task_goal_;           // qgoal of the navigation task being served
  bool has_task_goal_;       // false until the first plan() call

  unsigned int phase_id_;  // bumped whenever the phase target changes
  Metrics metrics_;        // instrumentation of the last plan() call
};
}  // namespace global_planner
#endif  // CG_RRT_PAPER_H
