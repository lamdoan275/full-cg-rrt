
#ifndef RRT_CUT_H
#define RRT_CUT_H

#include "rrt_astar.h"
#include "jump_point_search.h"


namespace global_planner
{

class RRTCut: public RRTAStar
{
public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  RRTCut(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, int k);
  /**
   * @brief RRT implementation
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * Nam add
   */
  bool escape(const Node& start, const Node& goal, Node& sub_goal);

  /**
   * Nam add
   */
  // bool _CheckNode(const Node& node);

protected:
  /**
   * @brief Caculate manhattan heuristic cost between 2 node
   * @param n1 node 1
   * @param n2 node 2
   * @return heuristic cost
   */
  double heuristic(const Node& n1, const Node& n2);

  double _getConfidence(const Node& goal);

  Node _getState(const Node& q, double confidence, double theta);

protected:
  std::shared_ptr<global_planner::GlobalPlanner> jps_plan;
  // Node sub_goal_;
};
}  // namespace global_planner
#endif  // RRT_H
