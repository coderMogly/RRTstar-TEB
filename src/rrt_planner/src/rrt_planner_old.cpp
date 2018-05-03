#include <pluginlib/class_list_macros.h>
#include "rrt_planner.h"
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <omplapp/apps/DynamicCarPlanning.h>
//#include <omplapp/config.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planner 
{
RRTPlanner::RRTPlanner() :
        costmap_(NULL), initialized_(false) {
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap) :
        costmap_(NULL), initialized_(false) {
    //initialize the planner
    initialize(name, costmap);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
if(!initialized_){
costmap_ = costmap_ros;
//costmap_ = costmap_ros_->getCostmap();

initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
}


bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

  if(!initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

plan.clear();

double start_x = start.pose.position.x;
double start_y = start.pose.position.y;
double goal_x = goal.pose.position.x;
double goal_y = goal.pose.position.y;

//ompl::app::DynamicCarPlanning setup;

// plan for dynamic car in SE(2)
//ompl::base::StateSpacePtr stateSpace(setup.getStateSpace());

//ompl::base::StateSpacePtr space(new ompl::base::ReedsSheppStateSpace);
ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

// set the bounds for the R^2 part of SE(2)
ompl::base::RealVectorBounds bounds(2);
bounds.setLow(-10);
bounds.setHigh(10);
//stateSpace->as<base::CompoundStateSpace>()->as<base::SE2StateSpace>(0)->setBounds(bounds);
space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
ompl::geometric::SimpleSetup setup(space);

// define start state
ompl::base::ScopedState<> scoped_start(space);
/*start[0] = start_x;
start[1] = start_y;
start[2] = start[3] = start[4] = 0.;*/
		scoped_start->as<ompl::base::SE2StateSpace::StateType>()->setX(start_x);
		scoped_start->as<ompl::base::SE2StateSpace::StateType>()->setY(start_y);
		scoped_start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);	

// define goal state
ompl::base::ScopedState<> scoped_goal(space);
/*goal[0] = goal_x;
goal[1] = goal_y;
goal[2] = 0;
goal[3] = goal[4] = 0.;*/

		scoped_goal->as<ompl::base::SE2StateSpace::StateType>()->setX(goal_x);
		scoped_goal->as<ompl::base::SE2StateSpace::StateType>()->setY(goal_y);
		scoped_goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);	

// set the start & goal states
setup.setStartAndGoalStates(scoped_start, scoped_goal, .5);

// optionally, set a planner
//setup.setPlanner(base::PlannerPtr(new control::EST(setup.getSpaceInformation())));
setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(setup.getSpaceInformation())));
//setup.setPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
//setup.setPlanner(base::PlannerPtr(new control::PDST(setup.getSpaceInformation())));
std::vector<double> cs(2);
cs[0] = cs[1] = 0.1;
setup.setup();
setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);


//std::cout<<"\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
   //std::vector<ompl::base::State *> poses;
// try to solve the prompl::baselem
if (setup.solve(40))
{
      // print the (approximate) solution path: print states along the path
      // and controls required to get from one state to the next
      ompl::geometric::PathGeometric ompl_path = setup.getSolutionPath();
      //path.interpolate(); // uncomment if you want to plot the path
      /*path.printAsMatrix(std::cout);
      if (!setup.haveExactSolutionPath())
      {
          std::cout << "Solution is approximate. Distance to actual goal is " <<
              setup.getPrompl::baselemDefinition()->getSolutionDifference() << std::endl;
      }*/
      //std::vector<ompl::base::State *> poses = soln.getStates();
std::vector<geometry_msgs::Pose2D> temp_plan_Pose2D;
geometry_msgs::Pose2D temp_pose;
unsigned int num_frames_inpath = ompl_path.getStateCount();
ROS_WARN("number of Path %d", num_frames_inpath);
		
		for(int i = 0; i < num_frames_inpath; i++)
		{
// 		get frame and transform it to Pose2D
//		OMPLStateSE2ToROSPose2D(ompl_path.states[i], temp_pose);		// 6/18/2014

			//OMPLStateSE2ToROSPose2D(ompl_path.getState(i), temp_pose);
		temp_pose.x = ompl_path.getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX();
		temp_pose.y = ompl_path.getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY();
		temp_pose.theta = ompl_path.getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
		temp_pose.theta = angles::normalize_angle(temp_pose.theta);			
			// output states for Debug
			ROS_WARN("Coordinates of %dth frame: (x, y, theta) = (%f, %f, %f).", i, temp_pose.x, temp_pose.y, temp_pose.theta);
			
			// and append them to plan
			temp_plan_Pose2D.push_back(temp_pose);
		}

/*void OMPLPlannerRRT::OMPLStateSE2ToROSPose2D(const ompl::base::State* ompl_state, geometry_msgs::Pose2D& pose2D)
	{
		pose2D.x = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
		pose2D.y = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
		pose2D.theta = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();	

		// normalize angle - just in case
		pose2D.theta = angles::normalize_angle(pose2D.theta);
		
		return;
	}

//const ompl::base::SE2StateSpace::StateType* state = states[0]->as<ompl::base::SE2StateSpace::StateType>();
//state = soln.getStates();
double x = soln.getX();
double y = soln->getY();
double theta = state->getYaw();
*/
for(int i = 0; i<num_frames_inpath; i++)
{

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = temp_plan_Pose2D.at(i).x; //Not sure if a vector will work as 2D matrix
  msg.pose.position.y = temp_plan_Pose2D.at(i).y;
  tf::Quaternion msg_quat = tf::createQuaternionFromYaw(temp_plan_Pose2D.at(i).theta);

  msg.pose.orientation.x = msg_quat.x();
  msg.pose.orientation.y = msg_quat.y();
  msg.pose.orientation.z = msg_quat.z();
  msg.pose.orientation.w = msg_quat.w();

  plan.push_back(msg);
}
}
//plan.push_back(goal);
return true;
} //makePlan
   
}; //namespace



