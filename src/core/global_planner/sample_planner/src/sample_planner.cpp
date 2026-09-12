/**
 * *********************************************************
 *
 * @file: sample_planner.cpp
 * @brief: Contains the sample planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2022-10-26
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>
#include <cmath>

#include "sample_planner.h"
#include "rrt.h"
#include "rrt_astar.h"
#include "rrt_cut.h"
#include "rrt_star.h"
#include "rrt_connect.h"
#include "informed_rrt.h"
#include "informed_rrt_star.h"
#include "quick_informed_rrt.h"

PLUGINLIB_EXPORT_CLASS(sample_planner::SamplePlanner, nav_core::BaseGlobalPlanner)

namespace sample_planner
{
/**
 * @brief  Constructor(default)
 */
SamplePlanner::SamplePlanner() : initialized_(false), g_planner_(NULL)
{
}
/**
 * @brief  Constructor
 * @param  name     planner name
 * @param  costmap  costmap pointer
 * @param  frame_id costmap frame ID
 */
SamplePlanner::SamplePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) : SamplePlanner()
{
  initialize(name, costmap, frame_id);
}

/**
 * @brief  Planner initialization
 * @param  name         planner name
 * @param  costmapRos   costmap ROS wrapper
 */
void SamplePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
}
/**
 * @brief  Planner initialization
 * @param  name     planner name
 * @param  costmap  costmap pointer
 * @param  frame_id costmap frame ID
 */
void SamplePlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
  if (!initialized_)
  {
    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);
    // costmap frame ID
    frame_id_ = frame_id;

    /*======================= static parameters loading ==========================*/
    private_nh.param("default_tolerance", tolerance_, 0.0);  // error tolerance
    private_nh.param("outline_map", is_outline_, false);     // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);       // obstacle inflation factor
    private_nh.param("expand_zone", is_expand_, false);      // whether publish expand zone or not

    // planner parameters
    // int sample_points;
    // double sample_max_d;
    private_nh.param("sample_points", sample_points, 1000);  // random sample points
    private_nh.param("sample_max_d", sample_max_d, 5.0);    // max distance between sample points

    // planner name
    // std::string planner_name;
    private_nh.param("planner_name", planner_name, (std::string) "rrt");
    if (planner_name == "rrt")
    {
      g_planner_ = std::make_shared<global_planner::RRT>(costmap, sample_points, sample_max_d);
    }
    else if (planner_name == "rrt_astar")
    { 
      // int k_nodes;
      private_nh.param("number_node_k", k_nodes, 5);  // optimization number of node
      g_planner_ = std::make_shared<global_planner::RRTAStar>(costmap, sample_points, sample_max_d, k_nodes);
      ROS_WARN("Planner name initial: %s", planner_name.c_str());

    }
    else if (planner_name == "rrt_cut")
    { 
      // int k_nodes;
      private_nh.param("number_node_k", k_nodes, 5);  // optimization number of node
      g_planner_ = std::make_shared<global_planner::RRTCut>(costmap, sample_points, sample_max_d, k_nodes);
      ROS_WARN("Planner name initial: %s", planner_name.c_str());
    }
    else if (planner_name == "rrt_star")
    {
      double optimization_r;
      private_nh.param("optimization_r", optimization_r, 10.0);  // optimization radius
      g_planner_ = std::make_shared<global_planner::RRTStar>(costmap, sample_points, sample_max_d, optimization_r);
    }
    else if (planner_name == "rrt_connect")
    {
      g_planner_ = std::make_shared<global_planner::RRTConnect>(costmap, sample_points, sample_max_d);
    }
    else if (planner_name == "informed_rrt")
    {
      double optimization_r;
      private_nh.param("optimization_r", optimization_r, 10.0);  // optimization radius
      g_planner_ = std::make_shared<global_planner::InformedRRT>(costmap, sample_points, sample_max_d, optimization_r);
    }
    else if (planner_name == "quick_informed_rrt")
    {
      int rewire_threads_n;
      double optimization_r, prior_set_r, step_ext_d, t_freedom;
      private_nh.param("optimization_r", optimization_r, 10.0);     // optimization radius
      private_nh.param("prior_sample_set_r", prior_set_r, 10.0);    // radius of priority circles set
      private_nh.param("rewire_threads_num", rewire_threads_n, 2);  // threads number of rewire process
      private_nh.param("step_extend_d", step_ext_d, 5.0);          // threads number of rewire process
      private_nh.param("t_distr_freedom", t_freedom, 1.0);         // freedom of t distribution
      g_planner_ = std::make_shared<global_planner::QuickInformedRRT>(
          costmap, sample_points, sample_max_d, optimization_r, prior_set_r, rewire_threads_n, step_ext_d, t_freedom);
    }
    else if (planner_name == "informed_rrt_star")
    {
      int informed_iterations;
      double search_radius, informed_threshold_cost;
      private_nh.param("search_radius", search_radius, 10.0);                     // rewire / goal-connection radius
      private_nh.param("informed_iterations", informed_iterations, 1000);         // phase-2 refinement iterations
      private_nh.param("informed_threshold_cost", informed_threshold_cost, 0.0);  // 0.0 = never early-stop
      g_planner_ = std::make_shared<global_planner::InformedRRTStar>(
          costmap, sample_points, sample_max_d, search_radius, informed_iterations, informed_threshold_cost);
    }
    else
    {
      // without this guard a misspelled name leaves g_planner_ as NULL and the
      // setFactor() call right below segfaults with no diagnostic at all
      ROS_ERROR("Unknown sample planner name: '%s'. SamplePlanner will stay uninitialized.", planner_name.c_str());
      return;
    }

    // pass costmap information to planner (required)
    g_planner_->setFactor(factor_);

    ROS_INFO("Using global sample planner: %s", planner_name.c_str());

    /*====================== register topics and services =======================*/
    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree", 1);
    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &SamplePlanner::makePlanService, this);

    if(planner_name == "rrt_cut"){
      local_sub_ = private_nh.subscribe<rosgraph_msgs::Log>("/rosout_agg", 100, &SamplePlanner::localPlanCallback, this);
      sub_goal_pub_ = private_nh.advertise<visualization_msgs::Marker>("sub_goal", 1);
    }

    // set initialization flag
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

/**
 * @brief plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully else false
 */
bool SamplePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}
bool SamplePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // start thread mutex
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());

  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  // clear existing plan
  plan.clear();

  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
              frame_id_.c_str(), goal.header.frame_id.c_str());
    return false;
  }
  if (start.header.frame_id != frame_id_)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
              frame_id_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  // get goal and start node coordinate tranform from world to costmap
  double wx = start.pose.position.x, wy = start.pose.position.y;
  unsigned int g_start_x, g_start_y, g_goal_x, g_goal_y;
  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  Node n_start(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  Node n_goal(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  // clear the cost of robot location
  g_planner_->getCostMap()->setCost(g_start_x, g_start_y, costmap_2d::FREE_SPACE);

  
  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  std::vector<Node> path;
  std::vector<Node> expand;

  if (planner_name == "rrt_cut")
  { 
    // Node sub_goal = n_goal, old_sub_goal = n_goal;
    // std::cout << "Address of costmap_ in global planner: " << g_planner_->getCostMap() << std::endl;
    global_planner::RRTCut rrt_cut_planner(g_planner_->getCostMap(), sample_points, sample_max_d, k_nodes);
    if(!rrt_cut_planner._isAnyObstacleInPath2(n_start, n_goal)){
            ROS_WARN("goal found");
            stuck_count = 0;
          }
    if(stuck_count < 5){
    
    bool sub_path_found = g_planner_->plan(n_start, n_goal, path, expand);
    if(sub_path_found){
      // if(!rrt_cut_planner._CheckNode(path.back()) || history_plan_.empty()){
      //   ROS_WARN("Check ok !");
        if (_getPlanFromPath(path, plan))
        {
          geometry_msgs::PoseStamped goalCopy = goal;
          goalCopy.header.stamp = ros::Time::now();
          plan.push_back(goalCopy); // add Goal
          history_plan_ = plan;
        }
        // }else if(history_plan_.size() > 0){
        //    plan = history_plan_;
        //    ROS_WARN("Local final node... Using history plan");
        // }
        else
          ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
    else if (history_plan_.size() > 0)
    {
      plan = history_plan_;
      ROS_WARN("Using history path.");
    }
    else
      ROS_ERROR("Failed to get a path.");
      // ROS_INFO("sub_start: %d", planner_name.c_str());

    // publish expand zone
    if (is_expand_)
      _publishExpand(expand);

    // publish visulization plan
    publishPlan(plan);
    return !plan.empty();
    }
    else{
      // escape trap 
      Node n_sub_goal;
      geometry_msgs::PoseStamped sub_goal;
      ROS_INFO("Stuck .... start escape");
      bool escape_found = rrt_cut_planner.escape(n_start, n_goal, n_sub_goal);
      g_planner_->map2World(n_sub_goal.x(), n_sub_goal.y(), sub_goal.pose.position.x, sub_goal.pose.position.y);
      publishSubGoal(sub_goal);
      
      if(sub_goal.pose.position.x == goal.pose.position.x && sub_goal.pose.position.y == goal.pose.position.y){
        stuck_count = 0;
      }

      if(escape_found){

        bool escape_path_found = g_planner_->plan(n_start, n_sub_goal, path, expand);
        if(escape_path_found){
          if(path.size() > 5){
          if (_getPlanFromPath(path, plan))
          {
            geometry_msgs::PoseStamped goalCopy = goal;
            goalCopy.header.stamp = ros::Time::now();
            plan.push_back(goalCopy); // add Goal
            history_plan_ = plan;
          }
          else
            ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
          }else{
            plan = history_plan_;
            ROS_WARN("Short path! Using history path.");
          }

        }
        else if (history_plan_.size() > 0)
        {
          plan = history_plan_;
          ROS_WARN("Using history path.");
        }
        else
          ROS_ERROR("Failed to get a path.");
        }else{
          ROS_ERROR("Failed to get a escape path.");
      }
      // ROS_INFO("sub_start: %d", planner_name.c_str());

      // publish expand zone
      if (is_expand_)
        _publishExpand(expand);

      // publish visulization plan
      publishPlan(plan);
      return !plan.empty();
    }

  }

  // ORTHER PLANNER
  else{

  bool path_found = g_planner_->plan(n_start, n_goal, path, expand);
  // ROS_INFO("Planninggg. %d", path_found);
  if (path_found)
  {
    // ROS_INFO("path size. %ld", path.size());
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
      history_plan_ = plan;
    }
    else
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
  }
  else if (history_plan_.size() > 0)
  {
    plan = history_plan_;
    ROS_WARN("Using history path.");
  }
  else
    ROS_ERROR("Failed to get a path.");

  // publish expand zone
  if (is_expand_)
    _publishExpand(expand);

  // publish visulization plan
  publishPlan(plan);
  return !plan.empty();
  }

  
}

/**
 * @brief  publish planning path
 * @param  path planning path
 */
void SamplePlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }
  // creat visulized path plan
  nav_msgs::Path gui_plan;
  gui_plan.poses.resize(plan.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < plan.size(); i++)
    gui_plan.poses[i] = plan[i];
  // publish plan to rviz
  plan_pub_.publish(gui_plan);
}

/**
 * @brief  regeister planning service
 * @param  req  request from client
 * @param  resp response from server
 */
bool SamplePlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;
  return true;
}

/**
 * @brief  publish expand zone
 * @param  expand  set of expand nodes
 */
void SamplePlanner::_publishExpand(std::vector<Node>& expand)
{
  ROS_DEBUG("Expand Zone Size:%ld", expand.size());

  // Initializes a Marker msg for a LINE_LIST
  visualization_msgs::Marker tree_msg;
  tree_msg.header.frame_id = "map";
  tree_msg.id = 0;
  tree_msg.ns = "tree";
  tree_msg.type = visualization_msgs::Marker::LINE_LIST;
  tree_msg.action = visualization_msgs::Marker::ADD;
  tree_msg.pose.orientation.w = 1.0;
  tree_msg.scale.x = 0.05;

  // Publish all edges
  for (auto node : expand)
    if (node.pid() != 0)
      _pubLine(&tree_msg, &expand_pub_, node.id(), node.pid());
}

/**
 * @brief  calculate plan from planning path
 * @param  path path generated by global planner
 * @param  plan plan transfromed from path
 * @return bool true if successful else false
 */
bool SamplePlanner::_getPlanFromPath(std::vector<Node> path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  std::string globalFrame = frame_id_;
  ros::Time planTime = ros::Time::now();
  plan.clear();

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    g_planner_->map2World((double)path[i].x(), (double)path[i].y(), wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}

/**
 *  @brief Publishes a Marker msg with two points in Rviz
 *  @param line_msg Pointer to existing marker object.
 *  @param line_pub Pointer to existing marker Publisher.
 *  @param id first marker id
 *  @param pid second marker id
 */
void SamplePlanner::_pubLine(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub, int id, int pid)
{
  // Update line_msg header
  line_msg->header.stamp = ros::Time::now();

  // Build msg
  geometry_msgs::Point p1, p2;
  std_msgs::ColorRGBA c1, c2;
  int p1x, p1y, p2x, p2y;

  g_planner_->index2Grid(id, p1x, p1y);
  g_planner_->map2World(p1x, p1y, p1.x, p1.y);
  p1.z = 1.0;

  g_planner_->index2Grid(pid, p2x, p2y);
  g_planner_->map2World(p2x, p2y, p2.x, p2.y);
  p2.z = 1.0;

  c1.r = 0.43;
  c1.g = 0.54;
  c1.b = 0.24;
  c1.a = 0.5;

  c2.r = 0.43;
  c2.g = 0.54;
  c2.b = 0.24;
  c2.a = 0.5;

  line_msg->points.push_back(p1);
  line_msg->points.push_back(p2);
  line_msg->colors.push_back(c1);
  line_msg->colors.push_back(c2);

  // Publish line_msg
  line_pub->publish(*line_msg);
}

void SamplePlanner::localPlanCallback(const rosgraph_msgs::Log::ConstPtr& msg)
{   
  if (msg->level == rosgraph_msgs::Log::WARN){

    std::string msg_text = msg->msg;
    if (msg_text.find("DWA planner failed to produce path.") != std::string::npos)
  {
    stuck_count++;
    ROS_INFO("Stuck count: %d", stuck_count);  
  }
  // else
  // {
  //   stuck_count = 0;  
  // }
  }
}

void SamplePlanner::publishSubGoal(const geometry_msgs::PoseStamped& sub_goal)
{
    // Create a marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sub_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = sub_goal.pose.position;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4; // Diameter of the sphere
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;  // alpha value

    sub_goal_pub_.publish(marker);
}

}  // namespace sample_planner



