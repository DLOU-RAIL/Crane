/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
//#include "/home/hill/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/moveit/moveit/moveit_planners/ompl/ompl_interface/include/moveit/ompl_interface/model_based_planning_context.h"
#include <moveit/ompl_interface/model_based_planning_context.h>

#include <boost/scoped_ptr.hpp>


// Display in Rviz tool
#include <ompl_visual_tools/ompl_visual_tools.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // 获得当前活动的场景，为碰撞检测做好准备
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef =   
        boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");  
  monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");  
  planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);  
  ps->getCurrentStateNonConst().update();  
  planning_scene::PlanningScenePtr scene_;
  scene_ = ps->diff();  
  scene_->decoupleParent();

  moveit::core::RobotModelConstPtr robot_model  = scene_->getRobotModel ();
  // We will now construct a loader to load a planner, by name. 
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    ROS_ERROR("planning_interface::PlannerManager:+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    ROS_ERROR("initialize planner instance:+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  
  req.group_name = "whole";
  //req.planner_id = "CBiMRRTConfigDefault";
  req.planner_id = "RRTConnectkConfigDefault";


  std::stringstream ss;
  ss << planner_plugin_name;
  ROS_ERROR_STREAM("BEFORE PLANNING:+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<ss.str());

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this 
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(scene_, req, res.error_code_);
  //ompl_interface::ModelBasedPlanningContext* pCon = boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(context);
  std::string strPlanner = boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(context)->getOMPLSimpleSetup()->getPlanner()->getName(); 
  ROS_INFO("Planner's Name is: %s", strPlanner.c_str());
  context->solve(res);
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }



  // 规划数据可视化
  ompl_visual_tools::OmplVisualToolsPtr visual_tools_;
  // Load the tool for displaying in Rviz
    visual_tools_.reset(new ompl_visual_tools::OmplVisualTools("base_footprint"));
    //visual_tools_->setSpaceInformation(si_);
    visual_tools_->setGlobalScale(100);

  //planning_scene_monitor::PlanningSceneMonitorPtr psm = getPlanningSceneMonitor();
  moveit::core::RobotModelConstPtr  moveit_robot_model = robot_model; //psm->getRobotModel();
  const robot_model::JointModelGroup* joint_model_group = moveit_robot_model->getJointModelGroup("whole");
  // Create a state space describing our robot's planning group
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(moveit_robot_model, joint_model_group);
    const ompl_interface::JointModelStateSpaceFactory factory;
    ompl_interface::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();

    visual_tools_->setStateSpace(model_state_space);

  const ob::PlannerDataPtr planner_data( new ob::PlannerData( boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(context)->getOMPLSimpleSetup()->getSpaceInformation() ) );
  boost::dynamic_pointer_cast<ompl_interface::ModelBasedPlanningContext>(context)->getOMPLSimpleSetup()->getPlannerData(*planner_data); 

  ros::Rate r(60);
    while(ros::ok())
    {
        visual_tools_->publishGraph(planner_data);
        r.sleep();
    }


  //END_TUTORIAL
  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
