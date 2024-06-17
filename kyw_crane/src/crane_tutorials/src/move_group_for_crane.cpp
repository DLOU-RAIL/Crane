
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "crane_location_regions.h"

bool generateValidStateByIK(const geometry_msgs::Pose& pose, planning_scene::PlanningScenePtr scene, robot_state::RobotState& rs)
{
    int nMaxSamples = 30000;
    bool bFound = false;
    int i = 0;
    const robot_state::JointModelGroup* joint_model_group = scene->getCurrentState().getJointModelGroup("whole");
    while(i<nMaxSamples)
    {
        // 求逆解
        bool bIK = rs.setFromIK(joint_model_group, pose, 2, 0.5);

        // 判断是否碰撞
        if(bIK && !scene->isStateColliding(rs, "whole"))
        {
            // // 同步到rviz中，用于调试
            // moveit_msgs::RobotState rs_msg;
            // robot_state::robotStateToRobotStateMsg(rs, rs_msg_);
            // jointPub_.publish(rs_msg_.joint_state);
            // ROS_INFO("%f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
            // rs.printStatePositions();
            // sleep(2);

            bFound = true;
            break;
        }
        i++;
    }

    return bFound;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_for_crane");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // ^^^^^
  static const std::string PLANNING_GROUP = "whole";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = -20;
  target_pose1.position.y = 0;
  target_pose1.position.z = 10;
  move_group.setPoseTarget(target_pose1);

    // 获得当前活动的场景，为碰撞检测做好准备
    planning_scene::PlanningScenePtr scene_;
    geometry_msgs::Pose initPose_;
    initPose_.position.x = 10;
    initPose_.position.y = 140;
    initPose_.position.z = 10.0;
    initPose_.orientation.w = 1;
    robot_state::RobotStatePtr rs_;
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef;
    monitor_ptr_udef.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
    ps->getCurrentStateNonConst().update();
    scene_ = ps->diff();
    scene_->decoupleParent();

  bool bInitFound = false;
  moveit::planning_interface::MoveGroupInterface* whole_group_ = new moveit::planning_interface::MoveGroupInterface("whole");
  robot_state::RobotState init_rs(*whole_group_->getCurrentState());
  bInitFound = generateValidStateByIK(initPose_, scene_, init_rs);
    if(bInitFound)
        ROS_INFO("Initial Robot State Found!!");
    else
        ROS_INFO("Initial Robot State NOT Found!!");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
