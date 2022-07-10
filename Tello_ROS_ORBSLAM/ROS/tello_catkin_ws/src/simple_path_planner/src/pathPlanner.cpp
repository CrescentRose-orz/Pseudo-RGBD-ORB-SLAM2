#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

class planner {
public:
    double height = 0;
    bool hasPlan = false;
	void setStart(double x, double y, double z)
	{
		std::cout<<"set startPoint to ("<<x<<" ,"<<y<<" ,"<<z<<" )"<<std::endl;
		ob::ScopedState<ob::SE3StateSpace> start(space);
		start->setXYZ(x,y,z);
		height = z;
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearStartStates();
		pdef->addStartState(start);
	}
	void setGoal(double x, double y, double z)
	{
		z = std::max(z,height);
		std::cout<<"set goal to ("<<x<<" ,"<<y<<" ,"<<z<<" )"<<std::endl;
		ob::ScopedState<ob::SE3StateSpace> goal(space);
		goal->setXYZ(x,y,z);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearGoal();
		pdef->setGoalState(goal);
		std::cout << "goal set to: " << x << " " << y << " " << z << std::endl;
	}
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
	{
		tree_obj = map;
	}
	// Constructor
	planner(double maxX = 5 ,double maxY = 5,double maxZ = 1,double minZ = 0)
	{
		//四旋翼的障碍物几何形状
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.08));
		//分辨率参数设置
		fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
		
		//解的状态空间
		space = ob::StateSpacePtr(new ob::SE3StateSpace());

		// create a start state
		ob::ScopedState<ob::SE3StateSpace> start(space);
		
		// create a goal state
		ob::ScopedState<ob::SE3StateSpace> goal(space);

		// set the bounds for the R^3 part of SE(3)
		// 搜索的三维范围设置
		ob::RealVectorBounds bounds(3);
		std::cout<<"receive bounds:"<<maxX<<" "<<maxY<<" "<<maxZ<<std::endl;
		bounds.setLow(0,-maxX);
		bounds.setHigh(0,maxX);
		bounds.setLow(1,-maxY);
		bounds.setHigh(1,maxY);
		bounds.setLow(2,minZ);
		bounds.setHigh(2,maxZ);

		space->as<ob::SE3StateSpace>()->setBounds(bounds);

		// construct an instance of  space information from this state space
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

		start->setXYZ(0,0,0);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// start.random();

		goal->setXYZ(0,0,0);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// goal.random();

		
	    // set state validity checking for this space
		si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

	    // set Optimizattion objective
		pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

		std::cout << "Initialized: " << std::endl;
	}
	// Destructor
	~planner()
	{
	}
	void replan(void)
	{
		if(!hasPlan){
			plan();
			return;
		}
		std::cout<<"REPLAN!"<<std::endl;
		std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;

		if(path_smooth->getStateCount () <= 2){
			std::cout<<"start plan"<<std::endl;
			plan();
		}
		else
		{
			std::cout<<"smooth?"<<std::endl;
			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
				if(!replan_flag)
					replan_flag = !isStateValid(path_smooth->getState(idx));
				else
					break;

			}
			if(replan_flag)
				plan();
			else
				std::cout << "Replanning not required" << std::endl;
		}
		
	}
	void plan(void)
	{
		
		std::cout<<"start planning!"<<std::endl;
	    // create a planner for the defined space
		og::InformedRRTstar* rrt = new og::InformedRRTstar(si);

		//设置rrt的参数range
		rrt->setRange(0.2);

		ob::PlannerPtr plan(rrt);

	    // set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

	    // perform setup steps for the planner
		plan->setup();

	    // print the settings for this space
		si->printSettings(std::cout);

		std::cout << "problem setting\n";
	    // print the problem settings
		pdef->print(std::cout);

	    // attempt to solve the problem within one second of planning time
		ob::PlannerStatus solved = plan->solve(1);

		if (solved)
		{
			hasPlan = true;
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);
	        // print the path to screen
	        // path->print(std::cout);


			nav_msgs::Path msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "map";

			for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
			{
				const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				geometry_msgs::PoseStamped pose;

//				pose.header.frame_id = "/world"

				pose.pose.position.x = pos->values[0];
				pose.pose.position.y = pos->values[1];
				pose.pose.position.z = pos->values[2];

				pose.pose.orientation.x = rot->x;
				pose.pose.orientation.y = rot->y;
				pose.pose.orientation.z = rot->z;
				pose.pose.orientation.w = rot->w;

				msg.poses.push_back(pose);

			}
			traj_pub.publish(msg);

			
	        //Path smoothing using bspline
			//B样条曲线优化
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);

			
			//Publish path as markers

			nav_msgs::Path smooth_msg;
			smooth_msg.header.stamp = ros::Time::now();
			smooth_msg.header.frame_id = "map";

			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
	                // cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
				
				geometry_msgs::PoseStamped point;

//				pose.header.frame_id = "/world"

				point.pose.position.x = pos->values[0];
				point.pose.position.y = pos->values[1];
				point.pose.position.z = pos->values[2];

				point.pose.orientation.x = rot->x;
				point.pose.orientation.y = rot->y;
				point.pose.orientation.z = rot->z;
				point.pose.orientation.w = rot->w;

				smooth_msg.poses.push_back(point);

				//std::cout << "Published marker: " << idx << std::endl;
			}

			vis_pub.publish(smooth_msg);
			// ros::Duration(0.1).sleep();
			

			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
		{
			std::cout << "No solution found" << std::endl;
		    hasPlan = false;
		}
	}
private:

	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	og::PathGeometric* path_smooth;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry> tree_obj;

	bool isStateValid(const ob::State *state)
	{
	    // cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	    // extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	    // extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		fcl::CollisionObject treeObj((tree_obj));
		fcl::CollisionObject aircraftObject(Quadcopter);

	    // check validity of state defined by pos & rot
		fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		aircraftObject.setTransform(rotation, translation);
		fcl::CollisionRequest requestType(1,false,1,false);
		fcl::CollisionResult collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return(!collisionResult.isCollision());
	}

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		// obj->setCostThreshold(ob::Cost(1.51));
		return obj;
	}

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		return obj;
	}

};


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
    //loading octree from binary
	// const std::string filename = "/home/xiaopeng/dense.bt";
	// octomap::OcTree temp_tree(0.1);
	// temp_tree.readBinary(filename);
	// fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	// convert octree to collision object
	std::cout<<"receiveing octomap"<<std::endl;
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	std::cout<<" octomap store"<<std::endl;
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	std::cout<<" fcl"<<std::endl;
	// Update the octree used for collision checking
	
	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
	std::cout<<" update map"<<std::endl;
	if (planner_ptr->hasPlan){
		planner_ptr->replan();
	}
	std::cout<<" replan"<<std::endl;
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void startCb(const geometry_msgs::PoseStamped::ConstPtr &msg, planner* planner_ptr)
{
static int count = 0;
	//++count; 一直重规划效果不佳
	planner_ptr->setStart(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	if (count == 30){
		count = 0;
		planner_ptr->plan();
	}
}

void goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg, planner* planner_ptr)
{
	std::cout<<"set a new goal"<<std::endl;
	planner_ptr->setGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	planner_ptr->plan();
}

int main(int argc, char **argv)
{
double bound_x,bound_y,bound_zMax,bound_zMin,bound_z;
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	std::string name_of_node_ = ros::this_node::getName();
    n.param<double>(name_of_node_+"/bound_x",bound_x,5);
	n.param<double>(name_of_node_+"/bound_y",bound_y,5);
	n.param<double>(name_of_node_+"/bound_z",bound_z,1);
	std::cout<<"get param"<<std::endl;
	// n.param<double>(name_of_node_+"/bound_z",bound_zMin,1);
    planner planner_object(bound_x,bound_y,bound_z,-bound_z);
	std::cout<<"planner init ok"<<std::endl;
	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/path_planner/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
	// ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/rovio/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
	ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&goalCb, _1, &planner_object));
	ros::Subscriber start_sub = n.subscribe<geometry_msgs::PoseStamped>("/orb_slam2_rgbd/pose", 1, boost::bind(&startCb, _1, &planner_object));
//orb_slam2_rgbd/pose
//	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	vis_pub = n.advertise<nav_msgs::Path>( "visualization_marker", 0 );
//	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
	traj_pub = n.advertise<nav_msgs::Path>("waypoints",1);
	
	std::cout << "path planer started.OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}
