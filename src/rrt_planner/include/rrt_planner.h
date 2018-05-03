#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>					// Jun/5/2014
#include <ompl/base/spaces/SE2StateSpace.h>			// Jun/5/2014
#include <ompl/base/PlannerData.h>							// Jun/5/2014
#include <ompl/geometric/SimpleSetup.h>					// Jun/10/2014
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/DiscreteMotionValidator.h>

//#include <ompl_ros_interface/OmplPlannerDiagnostics.h>
//#include <ompl/base/StateManifold.h>
//#include <ompl/base/manifolds/SE2StateManifold.h>
//#include <ompl/base/manifolds/RealVectorBounds.h>

namespace rrt_planner{
  /**
   * @class CarrotPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class RRTPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the CarrotPlanner
       */
      RRTPlanner();
      /**
       * @brief  Constructor for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

//			bool isStateValid(costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      ~RRTPlanner();

    private:
    	
    	ros::NodeHandle private_nh_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;								// 6/18/2014 () -> *
      double step_size_, min_dist_from_robot_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

			double inscribed_radius_, circumscribed_radius_, inflation_radius_;
			std::vector<geometry_msgs::Point> footprint_spec_;
			bool initialized_;
			
			//parameters to configure planner
			// parameter to flag whether path shall be interpolated (set to false for Elastic Bands)
			bool interpolate_path_;		
			// parameter to flag whether diagnostic and statistic msgs shall be published 
			bool publish_diagnostics_;
			// maximum cost for which footprint is still treated as collision free
			int max_footprint_cost_;
			// resolution of validity checking of robot motion 
			double relative_validity_check_resolution_;
			// parameter to set density of pathframes for interporation
			double max_dist_between_pathframes_;
			// parameter to switch between different planners provided through ompl
			std::string planner_type_;
			
			// Topics & Services
			// topic used to publish resulting plan for visualization
			ros::Publisher plan_pub_;
			// topic used to publish some diagnostic data about the results of the ompl
			ros::Publisher diagnostic_ompl_pub;
			// topic used to publish some statics about the planner plugin
			ros::Publisher stats_ompl_pub_;

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);
      
      bool isStateValid2DGrid(const ompl::base::State *state);
      
      bool interpolatePathPose2D(std::vector<geometry_msgs::Pose2D>& path);
      
      // Visualization
      // Publish a path for visualization purposes
      void publishPlan(std::vector<geometry_msgs::PoseStamped> path);
      
      // Read desired planner type from parameter server and set according ompl planner to simple setup
      void setPlannerType(ompl::geometric::SimpleSetup& simple_setup);
      
      // Converts an OMPL state of Type SE2 to a ROS Pose2D type
      void OMPLStateSE2ToROSPose2D(const ompl::base::State* ompl_state, geometry_msgs::Pose2D& pose2D);
      
      // converts an OMPL ScopedState of Type SE2 to a ROS Pose2D Type
      void OMPLScopedStateSE2ToROSPose2D(const ompl::base::ScopedState<> scoped_state, geometry_msgs::Pose2D& pose2D);
      
      // converts a ROS Pose2D type to an OMPL ScopedState of Type SE2
			void ROSPose2DToOMPLScopedStateSE2(ompl::base::ScopedState<>& scoped_state, const geometry_msgs::Pose2D pose2D);      
      // converts a frame of type Pose to type Pose2D
      void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D);
      
      // converts a frame of type Pose2D to type Pose
      void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D);

   };
};
#endif
