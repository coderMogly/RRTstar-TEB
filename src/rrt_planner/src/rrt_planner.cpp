#include "rrt_planner.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <iostream>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrt_planner {

	RRTPlanner::RRTPlanner()
	:costmap_ros_(NULL), initialized_(false){}
	
	RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	:costmap_ros_(NULL), initialized_(false)
	{
		initialize(name, costmap_ros);
	}
	
	RRTPlanner::~RRTPlanner()
	{
		delete world_model_;
	}
	
	void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){

			// create NodeHandle
      ros::NodeHandle private_nh("~/" + name);
      private_nh_ = private_nh;
      
      private_nh_.param("max_dist_between_pathframes", max_dist_between_pathframes_, 0.10);
      private_nh_.param("max_footprint_cost", max_footprint_cost_, 254);
      private_nh_.param("relative_validity_check_resolution", relative_validity_check_resolution_, 0.005);
      private_nh_.param("interpolate_path", interpolate_path_, false);
      private_nh_.param("publish_diagnostics", publish_diagnostics_, false);
      private_nh_.param("Inscribed_radius", inscribed_radius_, 0.2);				// 6/27/2014
      private_nh_.param("Circumscribed_radius", circumscribed_radius_, 0.25);		// 6/27/2014

/*** attention ***/
			plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);
/*****************/

			// get costmap
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();			
			world_model_ = new base_local_planner::CostmapModel(*costmap_);
					
			if(max_dist_between_pathframes_ <= 0.0)
			{
				ROS_WARN("Assigned Distance for interporation of path-frames invalid. Distance must be greater to 0. Distance set to default value: 0.10");
				max_dist_between_pathframes_ = 0.10;
			}
			
			initialized_ = true;
			}
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
  	ROS_INFO("Line 67");
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
    
    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;
    
    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

		// instantiate variables for statistics and diagnostics plotting
		ros::Time start_time, end_time;

//		if(publish_diagnostics_)
			start_time = ros::Time::now();
			
		ob::StateSpacePtr manifold(new ob::SE2StateSpace());								// 6/18/2014
		ROS_INFO("Line 97");
		// get bounds from worldmap and set it to bounds for the planner
		// as goal and map are set in same frame (checked above) we can directly
		// get the extensions of the statespace from the map-prms		
		ob::RealVectorBounds bounds(2);
		double map_upperbound, map_lowerbound;
ROS_INFO("Line 103");
		// get bounds for x coordinate
		map_upperbound = costmap_->getSizeInMetersX() - costmap_->getOriginX();	// costmap_->getOriginX();
		map_lowerbound = costmap_->getOriginX();											// map_upperbound - costmap_->getSizeInMetersX();
		bounds.setHigh(0, map_upperbound);
		bounds.setLow(0, map_lowerbound);
		ROS_WARN("Setting upper bound and lower bound of map x-coordinate to (%f, %f).", map_upperbound, map_lowerbound);
ROS_INFO("Line 110");
		// get bounds for y coordinate
		map_upperbound = costmap_->getSizeInMetersY() - costmap_->getOriginY();
		map_lowerbound = costmap_->getOriginY();											// map_upperbound - costmap_->getSizeInMetersY();		
		bounds.setHigh(1, map_upperbound);
		bounds.setLow(1, map_lowerbound);
		ROS_WARN("Setting upper bound and lower bound of map y-coordinate to (%f, %f).", map_upperbound, map_lowerbound);
ROS_INFO("Line 117");		
		// now set it to the planner
		manifold->as<ob::SE2StateSpace>()->setBounds(bounds);
		
		// now create instance to ompl setup
		og::SimpleSetup ss(manifold);
ROS_INFO("Line 123");		
		// set state validity checker
		ss.setStateValidityChecker(boost::bind(&RRTPlanner::isStateValid2DGrid, this, _1));ROS_INFO("Line 125");
		//ob::MotionValidatorPtr mv (new ob::DiscreteMotionValidator (ss.getSpaceInformation()));ROS_INFO("Line 126");
        //ss.getSpaceInformation ()->setMotionValidator (mv);ROS_INFO("Line 127");
		// get SpaceInformationPointer from simple_setup (initialized in makePlan routine)
		ob::SpaceInformationPtr si_ptr = ss.getSpaceInformation();ROS_INFO("Line 129");

		// set validity checking resolution
		si_ptr->setStateValidityCheckingResolution(relative_validity_check_resolution_);
		ROS_INFO("Line 133");
		// convert start and goal pose from ROS PoseStamped to ompl ScopedState for SE2
		// convert PoseStamped into Pose2D
		geometry_msgs::Pose2D start2D, goal2D;
		PoseToPose2D(start.pose, start2D);
		PoseToPose2D(goal.pose, goal2D);
		
		// before starting planner -> check whether target configuration is free
		// Goal position check
		int sample_costs = footprintCost(goal2D.x, goal2D.y, goal2D.theta);

		if ((sample_costs < 0.0) || (sample_costs > max_footprint_cost_))
		{
			ROS_ERROR("Collision on target: Planning aborted! Change target position.");
			return false;
		}
		
		// convert Pose2D to ScopedState
		ROS_DEBUG("Converting Start (%f, %f, %f) and Goal State (%f, %f, %f) to ompl ScopedState format",
		start2D.x, start2D.y, start2D.theta, goal2D.x, goal2D.y, goal2D.theta);
		
		// create a Scoped State according to above specified Manifold (SE2)
		ob::ScopedState<> ompl_scoped_state_start(manifold);
		
		// and set this state to the start pose
		ROSPose2DToOMPLScopedStateSE2(ompl_scoped_state_start, start2D);
		
		// check whether this satisfies the bounds of the manifold
		bool inBound = manifold->satisfiesBounds(ompl_scoped_state_start->as<ob::SE2StateSpace::StateType>());
		
		if(!inBound)
		{
			ROS_ERROR("Start Pose lies outside the bounds of the map - Aborting Planner ");
			return false;
		}
		
		// create a Scoped State according to above specified Manifold (SE2)
		ob::ScopedState<> ompl_scoped_state_goal(manifold);
		
		// and set this state to goal pose
		ROSPose2DToOMPLScopedStateSE2(ompl_scoped_state_goal, goal2D);
		
		// check whether this satisfies the bounds of the manifold
		inBound = manifold->satisfiesBounds(ompl_scoped_state_goal->as<ob::SE2StateSpace::StateType>());
		if(!inBound)
		{
			ROS_ERROR("Target Pose lies outside the bounds of the map - Aborting Planner ");
			return false;
		}
		
		// set start and goal state to planner
		ss.setStartAndGoalStates(ompl_scoped_state_start, ompl_scoped_state_goal);
		//si_ptr.setStartAndGoalStates(ompl_scoped_state_start, ompl_scoped_state_goal);
		
		// read desired planner-type from parameter server and set according planner to SimpleSetup
		//std::string default_planner("RRTstar");
		std::string default_planner("RRT");
		//std::string default_planner("EST");
		private_nh_.param("global_planner_type", planner_type_, default_planner);
		
		/*ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
		planner -> og::RRTstar::setGoalBias(0.05);		
		ss.setPlanner(planner);*/

		//og::RRTstar *r = new og::RRTstar(ss.getSpaceInformation());
		og::RRT *r = new og::RRT(ss.getSpaceInformation());
		//og::RRT *r = new og::RRT(ss.getSpaceInformation());
		
		//r -> og::RRTstar::setGoalBias(0.0);
		r -> og::RRT::setGoalBias(0.0);
		//r-> og::RRTstar::setRange(0.0005);
		ob::PlannerPtr planner(r);		
		ss.setPlanner(planner);
		//si_ptr.setPlanner(planner);
		//ROS_WARN("Range: %f", range +1);

		// finally --> plan a path (give ompl 1 second to find a valid path)
		ROS_DEBUG("Requesting Plan");		
		/************************************/
		// Execute the planning algorithm
		/************************************/
		// ob::PlannerStatus solved = ss.solve(1.0);	
		bool solved = ss.solve(5.0);	
		//bool solved = si_ptr.solve(1.0);	
		ROS_INFO("%d", solved);
		if (!solved)
		{
			ROS_WARN("No path found");
			return false;
		}
		// give ompl a chance to simplify the found solution
			ss.simplifySolution();
			//si_ptr.simplifySolution();

		// if path found -> get resulting path
		og::PathGeometric ompl_path(ss.getSolutionPath());		// 06/18/2014

		// convert into vector of pose2D
		ROS_DEBUG("Converting Path from ompl PathGeometric format to vector of PoseStamed");
		std::vector<geometry_msgs::Pose2D> temp_plan_Pose2D;
		geometry_msgs::Pose2D temp_pose;
/*		
		int num_frames_inpath = (int) ompl_path.states.size();		// 6/18/2014
*/
		unsigned int num_frames_inpath = ompl_path.getStateCount();

// represent number of path on the terminal.		
		ROS_WARN("number of Path %d", num_frames_inpath);
		
		for(int i = 0; i < num_frames_inpath; i++)
		{
// 		get frame and transform it to Pose2D
//		OMPLStateSE2ToROSPose2D(ompl_path.states[i], temp_pose);		// 6/18/2014

			OMPLStateSE2ToROSPose2D(ompl_path.getState(i), temp_pose);
			
			// output states for Debug
			ROS_WARN("Coordinates of %dth frame: (x, y, theta) = (%f, %f, %f).", i, temp_pose.x, temp_pose.y, temp_pose.theta);
			
			// and append them to plan
			temp_plan_Pose2D.push_back(temp_pose);
		}
		
		if(interpolate_path_)
		{
			ROS_DEBUG("Interpolating path to increase density of frames for local planning");
			// interpolate between frames to meet density requirement of local_planner
			bool ipo_success = interpolatePathPose2D(temp_plan_Pose2D);
			if(!ipo_success)
			{
				ROS_ERROR("Something went wrong during interpolation. Probably plan empty. Aborting!");
				return false;
			}
			int num_frames_inpath = (int) temp_plan_Pose2D.size();
			// ROS_DEBUG("Interpolated Path has %d frames", num_frames_inpath);
			ROS_DEBUG("Interpolated Path has %d frames", num_frames_inpath);	
		}
		
		// convert into vector of PoseStamped
		std::vector<geometry_msgs::PoseStamped> temp_plan;
		geometry_msgs::PoseStamped temp_pose_stamped;
		
		temp_plan.push_back(start);
		for(int i = 0; i < num_frames_inpath; i++)
		{
			// set Frame to PoseStamped
			ros::Time plan_time = ros::Time::now();
			
			// set header
			temp_pose_stamped.header.stamp = plan_time;
			temp_pose_stamped.header.frame_id = costmap_ros_->getGlobalFrameID();
			
			// convert Pose2D to pose and set to Pose of PoseStamped
			// Quaternion 
			Pose2DToPose(temp_pose_stamped.pose, temp_plan_Pose2D[i]);
			
			// append to plan
			temp_plan.push_back(temp_pose_stamped);
			//plan.push_back(temp_pose_stamped);
		}
		temp_plan.push_back(goal);
		ROS_INFO("Global planning finished: Path Found.");
		plan = temp_plan;
		
		publishPlan(plan);
		
		if(publish_diagnostics_)
		{
			// compose msg with stats

			// set end time for logging of planner statistics
			end_time = ros::Time::now();
			ros::Duration planning_duration = end_time -start_time;
		}
		// publish the Plan for visualization purpose
		
		// and returtn with true
		return true;
	}

	// we need to take the footprint of the robot into account when we calculate cost to obstacles
	// most important
	double RRTPlanner::footprintCost(double x_i, double y_i, double theta_i)
	{
		if(!initialized_)
		{
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return -1.0;
		}
		// if we have no footprint... do nothing
		
		// following the CarrotPlanner 6/28/2014
		std::vector<geometry_msgs::Point> footprint_spec_ = costmap_ros_->getRobotFootprint();
		if(footprint_spec_.size() < 3)
			return -1.0;
		
		// build the oriented footprint
		double cos_th = cos(theta_i);
		double sin_th = sin(theta_i);
		std::vector<geometry_msgs::Point> oriented_footprint;
		for(unsigned int i = 0; i < footprint_spec_.size(); i++){
			geometry_msgs::Point new_pt;
			new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
			new_pt.y = y_i + (footprint_spec_[i].x * sin_th - footprint_spec_[i].y * cos_th);
			oriented_footprint.push_back(new_pt);
		}
		
		geometry_msgs::Point robot_position;
		robot_position.x = x_i;
		robot_position.y = y_i;
		
		// check if the footprint is legal
		double footprint_cost = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
		return footprint_cost;
	}
	
	bool RRTPlanner::isStateValid2DGrid(const ompl::base::State *state)
	{
		geometry_msgs::Pose2D checked_state;
		double costs = 0.0;
		
		// transform ompl::base::state back to ros Pose2D
		OMPLStateSE2ToROSPose2D(state, checked_state);
		
		// check the pose using the footprint_cost check
		costs = footprintCost(checked_state.x, checked_state.y, checked_state.theta);
		//ROS_INFO("%f", costs);
		if( (costs >= 0) && (costs < max_footprint_cost_) )
		{	ROS_INFO("STATE: %f, %f, %f", costs, checked_state.x, checked_state.y);
			return true;
		}ROS_INFO("INVALID STATE: %f, %f, %f", costs, checked_state.x, checked_state.y);
		return false;
	}
	
	bool RRTPlanner::interpolatePathPose2D(std::vector<geometry_msgs::Pose2D>& path)
	{
		std::vector<geometry_msgs::Pose2D> ipoPath;
		geometry_msgs::Pose2D last_frame, curr_frame, diff_frame, temp_frame;
		double frame_distance, num_insertions;
		int path_size = path.size();
		
		// check whether planner is already initialized
		if(!initialized_)
		{
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return false;
		}
		
		// check whether path is correct - at least 2 Elements
		if(path_size < 2)
		{
			ROS_ERROR("Path is not valid. It has only %d Elements. Interpolation not possible. Aborting.", path_size);
			return false;
		}
		
		// init plan with start frame
		ipoPath.push_back(path[0]);
		
		// make sure plan is dense enough to be processed by local planner
		for(int i = 1; i < path_size; i++)
		{
			// check whether current frame is close enough to last frame --> otherwise insert interpolated frames
			last_frame = ipoPath[(ipoPath.size()-1)];
			curr_frame = path[i];
			
			// calc distance between frames
			diff_frame.x = curr_frame.x - last_frame.x;
			diff_frame.y = curr_frame.y - last_frame.y;
			diff_frame.theta = curr_frame.theta - last_frame.theta;
			// normalize angle
			diff_frame.theta = angles::normalize_angle(diff_frame.theta);
			
			frame_distance = sqrt( diff_frame.x*diff_frame.x + diff_frame.y*diff_frame.y );
			
			// insert frames until path is dense enough
			if(frame_distance > max_dist_between_pathframes_)
			{
				// just in case --> insert one frame more than neccesarry
				num_insertions = ceil(frame_distance/max_dist_between_pathframes_);
				
				diff_frame.x = diff_frame.x/(num_insertions + 1.0);
				diff_frame.y = diff_frame.y/(num_insertions + 1.0);
				diff_frame.theta = diff_frame.theta/(num_insertions + 1.0);
				for(int j = 1; j <= (int)num_insertions; j++)
				{
					temp_frame.x = last_frame.x + j*diff_frame.x;
					temp_frame.y = last_frame.y + j*diff_frame.y;
					temp_frame.theta = last_frame.theta + j*diff_frame.theta;
					// normalize angle
					temp_frame.theta = angles::normalize_angle(temp_frame.theta);
					
					// append frame to interpolated path
					ipoPath.push_back(temp_frame);
				}
			}
			
			// finally insert frame from path
			ipoPath.push_back(curr_frame);
		}
		
		// done --> copy ipoPath to reference-variable and return with true
		path = ipoPath;
		return true;
	}
	
	// Visualization
	
	void RRTPlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> path)
	{
		// check whether planner is already initialized --> should be the case anyway but better be sure
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
			return;
		}
		
		// check whether there really is a path --> given an empty path we won't do anything
		if(path.empty())
		{
			ROS_INFO("Plan is empty - Nothing to display");
			return;
		}
		
		// create a message for the plan
		nav_msgs::Path gui_path;
		gui_path.poses.resize(path.size());
		gui_path.header.frame_id = path[0].header.frame_id;
		gui_path.header.stamp = path[0].header.stamp;
		
		// Extract the plan in world coordinates, we assume the path is all in the same frame
		for(unsigned int i=0; i < path.size(); i++)
		{
			gui_path.poses[i] = path[i];
		}
			plan_pub_.publish(gui_path);
	}
	
	void RRTPlanner::OMPLStateSE2ToROSPose2D(const ob::State* ompl_state, geometry_msgs::Pose2D& pose2D)
	{
		pose2D.x = ompl_state->as<ob::SE2StateSpace::StateType>()->getX();
		pose2D.y = ompl_state->as<ob::SE2StateSpace::StateType>()->getY();
		pose2D.theta = ompl_state->as<ob::SE2StateSpace::StateType>()->getYaw();	

		// normalize angle - just in case
		pose2D.theta = angles::normalize_angle(pose2D.theta);
		
		return;
	}
	
	void RRTPlanner::ROSPose2DToOMPLScopedStateSE2(ob::ScopedState<>& scoped_state, const geometry_msgs::Pose2D pose2D)
	{
		scoped_state->as<ob::SE2StateSpace::StateType>()->setX(pose2D.x);
		scoped_state->as<ob::SE2StateSpace::StateType>()->setY(pose2D.y);
		scoped_state->as<ob::SE2StateSpace::StateType>()->setYaw(pose2D.theta);	
		
		return;
	}
	
	void RRTPlanner::PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D)
	{
		// use tf-pkg to convert angles
		tf::Pose pose_tf;
		
		// convert geometry_msgs::PoseStamped to tf::Pose
		tf::poseMsgToTF(pose, pose_tf);
		
		// now get Euler-Angles from pose_tf
		double useless_pitch, useless_roll, yaw;
		pose_tf.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
		///Volumes/NO NAME/rrt_planner/src/rrt_planner.cpp
		// normalize angle
		yaw = angles::normalize_angle(yaw);
		
		// and set to pose2D
		pose2D.x = pose.position.x;
		pose2D.y = pose.position.y;
		pose2D.theta = yaw;
		
		return;
	}
	
	void RRTPlanner::Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D)
	{
		// use tf-pkg to convert angles
		tf::Quaternion frame_quat;
		
		// transform angle from euler-angle to quaternion representation
		frame_quat = tf::createQuaternionFromYaw(pose2D.theta);
		
		// set position
		pose.position.x = pose2D.x;
		pose.position.y = pose2D.y;
		pose.position.z = 0.0;
		
		// set quaternion
		pose.orientation.x = frame_quat.x();
		pose.orientation.y = frame_quat.y();
		pose.orientation.z = frame_quat.z();
		pose.orientation.w = frame_quat.w();
		
		return;
	}
		
};
