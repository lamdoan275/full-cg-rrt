
#ifndef RRT_CUT_H
#define RRT_CUT_H

#include "rrt_astar.h"
#include "graph_planner/include/jump_point_search.h"

namespace global_planner
{

class SubGoalRRT: public RRTAStar
{
public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  SubGoalRRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, int k);
  /**
   * @brief RRT implementation
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
   /**
   * @brief Detect whether current node has forced neighbor or not
   * @param cur_node  current node
   * @param motion the motion that current node executes
   * @return true if current node has forced neighbor else false and force_neighbor
   */
  bool _detectForceNeighbor(const Node& cur_node, const Node& motion, Node& force_neighbor);

  /**
   * @brief Detect whether current node has forced neighbor or not
   * @param cur_node  current node
   * @param motions the motion that current node executes
   * @return vector contain neighbor nodes
   */
  std::vector<Node> _findForceNeighbors(const Node& cur_node, const Node& motion);

  /**
   * @brief get best motion from jump_init to goal
   * @param motions current node
   * @param start, goal start and goal 
   * @return best motion from jump_int to goal
   */
  Node _getConfident(const std::vector<Node>& motions, const Node& start, const Node& goal);

 /**
   * @brief normalize the vector
   * @param vector vecto to normalize
   * @return vector has normalized in motions
   */
  Node _getSubGoal(const Node& cur_goal) const;
  
// protected:
//   int k_;  // optimization number of nodes
};
}  // namespace global_planner
#endif  // RRT_H
