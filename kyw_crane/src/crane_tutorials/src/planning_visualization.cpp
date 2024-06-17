// ROS
#include <ros/ros.h>

// Display in Rviz tool
#include <ompl_visual_tools/ompl_visual_tools.h>

// OMPL
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// Boost
#include <boost/pointer_cast.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;


// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "planning_visualization");
  ROS_INFO( "OMPL Visual Tools Demo ----------------------------------------- " );

  // Load the tool for displaying in Rviz
    visual_tools_.reset(new ompl_visual_tools::OmplVisualTools("base_footprint"));
    //visual_tools_->setSpaceInformation(si_);
    visual_tools_->setGlobalScale(100);

  planning_scene_monitor::PlanningSceneMonitorPtr psm = getPlanningSceneMonitor();
  moveit::core::RobotModelConstPtr  moveit_robot_model = psm->getRobotModel();
  const robot_model::JointModelGroup* joint_model_group = moveit_robot_model->getJointModelGroup("whole");
  // Create a state space describing our robot's planning group
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(moveit_robot_model, joint_model_group);
    const ompl_interface::JointModelStateSpaceFactory factory;
    ompl_interface::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();

    visual_tools_->setStateSpace(model_state_space);


    // std::vector<ompl::base::PlannerDataPtr> paths;
    // simple_setup.getAllPlannerDatas(paths);

    // // Get tip links for this setup
    // std::vector<const robot_model::LinkModel*> tips;
    // joint_model_group_->getEndEffectorTips(tips);

    // bool show_trajectory_animated = true;

    ompl::base::PlannerDataPtr planner_data = ;
    ros::Rate r(60);
    while(ros::ok())
    {
        visual_tools_->publishGraph(planner_data);
        r.sleep();
    }
}