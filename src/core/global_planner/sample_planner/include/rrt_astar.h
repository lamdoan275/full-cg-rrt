
#ifndef RRT_ASTAR_H
#define RRT_ASTAR_H

#include "rrt.h"

namespace global_planner
{

class RRTAStar : public RRT
{
public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  RRTAStar(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, int k);
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
    * @brief Get nearest node by Nam
    * @param list  samplee list
    * @param node  sample node
    * @return nearest node
    */
  Node _getNearestNode(std::unordered_map<int, Node>& list, const Node& node);

   /**
    * @brief Get new node by Nam
    * @param nearest_node  nearest node in sample list
    * @param node  sample node
    * @return nearest node
    */
  Node _getNewNode(Node& nearest_node, const Node& node);

  // double heuristic(const Node& n1, const Node& n2);


protected:
  int k_;  // optimization number of nodes
};
}  // namespace global_planner
#endif  // RRT_H
