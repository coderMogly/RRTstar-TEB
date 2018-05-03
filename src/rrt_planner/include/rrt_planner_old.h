#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <vector>

using std::string;

namespace rrt_planner {

 class RRTPlanner : public nav_core::BaseGlobalPlanner {
 public:

  RRTPlanner();
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );

  costmap_2d::Costmap2DROS* costmap_;
  bool initialized_;
  };
 };
 #endif

