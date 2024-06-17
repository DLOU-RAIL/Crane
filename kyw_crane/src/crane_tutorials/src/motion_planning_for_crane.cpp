

//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>  

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include "crane_location_regions.h"


moveit_msgs::CollisionObject* loadMeshFromFile(std::string strMeshFile, std::string strName)
{
  std::string mesh_path = "file://" + strMeshFile;
  shapes::Shape* mesh = shapes::createMeshFromResource(mesh_path);  // make sure its prepended by file://
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    return NULL;
  }
  // Create collision message
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  moveit_msgs::CollisionObject* pColObj = new moveit_msgs::CollisionObject();
  pColObj->header.stamp = ros::Time::now();
  pColObj->header.frame_id = "base_link";
  pColObj->id = strName;
  pColObj->operation = moveit_msgs::CollisionObject::ADD;
  pColObj->mesh_poses.resize(1);
  pColObj->mesh_poses[0] = pose;
  pColObj->meshes.resize(1);
  pColObj->meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

  return pColObj;
}

bool planningForBase(std::vector<double>& startConf, std::vector<double>& targetConf, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
  moveit::planning_interface::MoveGroupInterface group("base");
  // 设置工作空间范围
  group.setWorkspace(-60, -50, -1, 100, 50, 100);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(20);
  group.setNumPlanningAttempts(3);

  // 设置起始位形
  robot_state::RobotState robot_state(*group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("base");
  robot_state.setJointGroupPositions(joint_model_group, startConf);
  group.setStartState(robot_state);

  // 设置目标位形
  group.setJointValueTarget(targetConf);

  // 求解规划问题
  bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  return success;
}

bool planningForUpper(std::vector<double>& startConf, std::vector<double>& targetConf, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
  moveit::planning_interface::MoveGroupInterface group("upper");
  // 设置工作空间范围
  group.setWorkspace(-60, -50, -1, 100, 50, 100);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(20);
  group.setNumPlanningAttempts(5);

  // // 设置起始位形
  robot_state::RobotState robot_state(*group.getCurrentState());
  robot_state.printStatePositions();
  int n = robot_state.getVariableCount ();
  ROS_INFO("n=%d",n);
  double joint_values[10] = {80, -15.5, 0.0, 0.0, 0.0, 0.0, 1.085, -1.085, 0, 0};
  robot_state.setVariablePositions(joint_values);
  // const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("upper");
  // robot_state.setJointGroupPositions(joint_model_group, startConf);
  group.setStartState(robot_state);
  //group.setStartStateToCurrentState();

  // 设置目标位形
  group.setJointValueTarget(targetConf);

  // 求解规划问题
  bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  return success;
}

// void printPlanInfo(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
// {
//   ROS_INFO("Planning Time: %f", my_plan.planning_time_);
//   // 输出关节轨迹
//   trajectory_msgs::JointTrajectory traj = my_plan.trajectory_.joint_trajectory;
//   int nLen = traj.points.size();
//   ROS_INFO("nLen=%d", nLen);
//   ROS_INFO("nJoints=%d", traj.points[0].positions.size());
//   for(int i = 0; i < nLen; i++)
//   {
//     ROS_INFO("(%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f)",
//     traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],
//     traj.points[i].positions[5],traj.points[i].positions[6],traj.points[i].positions[7],traj.points[i].positions[8],traj.points[i].positions[9]);
//   }
// }

bool group_state_validity_callback(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group, const double* joint_group_variable_values)
{
  robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
  double dx = joint_group_variable_values[0] - 50;
  double dy = joint_group_variable_values[0] + 31.9;
  double da = atan2(dy, dx);
  double dr2 = dx*dx+dy*dy;
  // if( /*dr2 > 18*18 && dr2 < 32*32 && */da > 0.0 && da < 1.57)
  // {
  //   return true;
  // }
  // else
  // {
  //   return false;
  // }
  ROS_INFO("dx=%f, dy=%f, da=%f, dr = %f", dx, dy, da, sqrt(dr2));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_planning_for_crane");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  CraneLocationRegions CLR;
  CLR.liftObjPose_.dX = 50.85;
  CLR.liftObjPose_.dY = -31.9;
  CLR.liftObjPose_.dZ = 39.52;
  CLR.liftObjPose_.dFai = 0.0;
  CLR.addAnnularSector(18.0, 32.0, -3.14, 3.14, -3.14, 3.14);
  std::vector<double> joints;
  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
  moveit::planning_interface::MoveGroupInterface whole_group("whole");
  robot_state::RobotState robot_state(*whole_group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("whole");

  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef;
  monitor_ptr_udef.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));   
    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");  
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);  
    ps->getCurrentStateNonConst().update();  
    planning_scene::PlanningScenePtr scene = ps->diff();  
    scene->decoupleParent();  

  while(ros::ok())
  {
  CLR.directSampling(joints);
  robot_state.setJointGroupPositions(joint_model_group, joints);

  scene->setCurrentState(robot_state);
  robot_state::RobotState& current_state = scene->getCurrentStateNonConst();  
  bool bValid = scene->isStateValid(current_state, "whole");  
  //bool bValid = planning_scene.isStateColliding(robot_state, "whole");
  if(bValid)
    ROS_INFO("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO COLLISION");
  else
    ROS_ERROR("COLLISION!!");

  // 同步到rviz中
  moveit_msgs::RobotState rs_msg;
  robot_state::robotStateToRobotStateMsg(robot_state, rs_msg);
  joint_pub.publish(rs_msg.joint_state);
  //robot_state.printStatePositions();
  sleep(2);
  }

/*
  moveit::planning_interface::MoveGroupInterface base_group("whole");
  robot_state::RobotState robot_state(*base_group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("whole");
  geometry_msgs::Pose pose; 
    pose.position.x = 50;
    pose.position.y = -31.9;
    pose.position.z = 9.52;
    pose.orientation.w = 1;


ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    std::string str[3] = {"base_x_joint", "base_y_joint", "base_theta_joint"};
    std::vector< std::string > js = std::vector<std::string>(str, str+3);
    joint_model_group->setRedundantJoints(js);
    kinematics::KinematicsQueryOptions options;
    options.lock_redundant_joints = true;

    std::vector< double > gstate(10, 0.0);
    // 随机数生成器
    random_numbers::RandomNumberGenerator rng_;
    while(ros::ok())
    {
      robot_state.copyJointGroupPositions(joint_model_group, gstate);
      double r = rng_.uniformReal( 18, 32 );
    double theta = rng_.uniformReal( 0, 1.57 );
    double alpha = rng_.uniformReal( -3.14, 3.14 );
      gstate[0] = pose.position.x + r * cos( theta );  // 起重机位置x
      gstate[1] = pose.position.y + r * sin( theta );  // 起重机位置y
      gstate[2] = alpha;  // 起重机下车方向角
      robot_state.setJointGroupPositions(joint_model_group, gstate);
      if( robot_state.setFromIK(joint_model_group, pose, 2, 0.1, group_state_validity_callback, options))
      {
        moveit_msgs::DisplayRobotState msg;
        moveit_msgs::RobotState rs_msg;
        robot_state::robotStateToRobotStateMsg(robot_state, rs_msg);
        joint_pub.publish(rs_msg.joint_state);
        //robot_state_publisher_.publish(msg);
        robot_state.printStatePositions();
      }
      sleep(1);
    }
*/

/*
  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  bool success;
  // ----------------------对下车进行运动规划---------------------------------//
  std::vector<double> baseStartConf(3, 0.0), baseTargetConf(3, 0.0);
  baseStartConf[0] = -28.04;
  baseStartConf[1] = -24;
  baseStartConf[2] = 0.5;
  baseTargetConf[0] = 80;
  baseTargetConf[1] = -15.5;
  baseTargetConf[2] = 0.0;
  moveit::planning_interface::MoveGroupInterface::Plan base_plan;   // 其中的joint_trajectory只包含所规划的几个自由度
  moveit::planning_interface::MoveGroupInterface base_group("base");
  // 设置工作空间范围
  base_group.setWorkspace(-60, -50, -1, 100, 50, 100);
  base_group.setPlannerId("PRMkConfigDefault");
  base_group.setPlanningTime(5);
  base_group.setNumPlanningAttempts(3);
  // 设置起始位形
  robot_state::RobotState robot_state(*base_group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("base");
  robot_state.setJointGroupPositions(joint_model_group, baseStartConf);
  base_group.setStartState(robot_state);
  // 设置目标位形
  base_group.setJointValueTarget(baseTargetConf);
  // 求解规划问题
  success = base_group.plan(base_plan);

  if(success)
  {
    ROS_INFO("FOUND A PATH FOR CRANE'S BASE!!");
    display_trajectory.trajectory_start = base_plan.start_state_;
    display_trajectory.trajectory.push_back(base_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    ROS_INFO("Planning Time for Base: %f", base_plan.planning_time_);
    sleep(10.0);
  }
  else
  {
    ROS_INFO("FAILURE!!");
  }

  //printPlanInfo(base_plan);

  // ----------------------对上车进行运动规划---------------------------------//
  moveit::planning_interface::MoveGroupInterface upper_group("upper");
  moveit::planning_interface::MoveGroupInterface::Plan upper_plan;
  if(success)
  {
    // 设置工作空间范围
    upper_group.setWorkspace(-60, -50, -1, 100, 50, 100);
    upper_group.setPlannerId("RRTConnectkConfigDefault");
    upper_group.setPlanningTime(20);
    upper_group.setNumPlanningAttempts(5);
    // 设置起始位形
    double joint_values[10] = {80, -15.5, 0.0, 0.0, 0.0, 0.0, 1.085, -1.085, 0, 0};
    robot_state.setVariablePositions(joint_values);
    upper_group.setStartState(robot_state);
    // 设置目标位形
    // std::vector<double> upperTargetConf(5, 0.0);
    // upperTargetConf[0] = -0.6;           //(80.0, -15.5, 0.0, -0.6, 1.15, -1.15， 41.02, 0.0)
    // upperTargetConf[1] = 1.15;
    // upperTargetConf[2] = -1.15;
    // upperTargetConf[3] = 41.02;
    // upperTargetConf[4] = 0.0;
    // upper_group.setJointValueTarget(upperTargetConf);
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 99.85;
    target_pose1.position.y = -31.9;
    target_pose1.position.z = 9.52;
    //upper_group.setPoseTarget(target_pose1);
    upper_group.setApproximateJointValueTarget(target_pose1,"hook_link");
    // 求解规划问题
    success = upper_group.plan(upper_plan);
    if(success)
    {
      ROS_INFO("FOUND A PATH FOR CRANE'S UPPER!!");
      display_trajectory.trajectory.push_back(upper_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      ROS_INFO("Planning Time for Upper: %f", upper_plan.planning_time_);
      sleep(5);
    }
    else
    {
      ROS_INFO("FAILURE!!");
    }

  }
*/


//   ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
//   while(planning_scene_diff_publisher.getNumSubscribers() < 1)
//   {
//     ros::WallDuration sleep_t(0.5);
//     sleep_t.sleep();
//   }


//   /* This sleep is ONLY to allow Rviz to come up */
//   // sleep(20.0);
  
//   moveit::planning_interface::MoveGroupInterface group("base");
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = false;

//   // We will use the :planning_scene_interface:`PlanningSceneInterface`
//   // class to deal directly with the world.
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

//   // (Optional) Create a publisher for visualizing plans in Rviz.
//   ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//   moveit_msgs::DisplayTrajectory display_trajectory;

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


//   // 设置工作空间范围
//   group.setWorkspace(-60, -50, -1, 100, 50, 100);
//   group.setPlannerId("RRTConnectkConfigDefault");
//   group.setPlanningTime(20);
//   // // setPlannerParams (const std::string &planner_id, const std::string &group, const std::map< std::string, std::string > &params, bool bReplace=false)
//   // std::map< std::string, std::string > params;
//   // params.insert(std::pair<std::string, std::string>("range", "0.3")); 
//   // group.setPlannerParams("RRTConnectkConfigDefault", "base", params, true);

//   // // 设置吊装环境
//   // moveit_msgs::CollisionObject* pColObj = NULL;
//   // std::vector<moveit_msgs::CollisionObject> collision_objects;  

//   // moveit_msgs::PlanningScene planning_scene;
//   // std::string fileName[3] = {"B.obj", "C.obj", "D.obj"};
//   // for(int i = 0; i < 3; i++)
//   // {
//   //   pColObj = loadMeshFromFile("/home/hill/3DEnvs/18-2-1_models/envs/" + fileName[i], fileName[i]);
//   //   planning_scene.world.collision_objects.push_back(*pColObj);
//   //   collision_objects.push_back(*pColObj); 
//   //   delete pColObj;
//   //   pColObj = NULL;
//   // }
//   // planning_scene.is_diff = true;
//   // // 法1：通过msg添加环境碰撞模型
//   // //planning_scene_diff_publisher.publish(planning_scene);          
//   // // 法2：通过服务添加环境碰撞模型
//   // ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
//   // planning_scene_diff_client.waitForExistence();
//   // // and send the diffs to the planning scene via a service call:
//   // moveit_msgs::ApplyPlanningScene srv;
//   // srv.request.scene = planning_scene;
//   // planning_scene_diff_client.call(srv);
//   // // 法3：通过planning_scene_interface添加环境碰撞模型
//   // //planning_scene_interface.addCollisionObjects(collision_objects);
  
//   // /* Sleep so we have time to see the object in RViz */
//   // sleep(2.0);

//   // 设置起始位形
//   std::vector<double> joint_values(3, 0.0);
//   joint_values[0] = -28.04;
//   joint_values[1] = -24;
//   joint_values[2] = 0.5;
//   // joint_values[3] = -2.0;
//   // joint_values[4] = 1.2;
//   // joint_values[5] = -1.2;
//   // joint_values[6] = 45.25;
//   // joint_values[7] = 0.0;
//   robot_state::RobotState robot_state(*group.getCurrentState());
//   //robot_state.printStateInfo();
//   const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("base");
//   robot_state.setJointGroupPositions(joint_model_group, joint_values);
//   //robot_state.printStatePositions();
//   group.setStartState(robot_state);
//   //group.setStartStateToCurrentState();


//   // Planning to a joint-space goal 
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   std::vector<double> group_variable_values;              // 包含 mimic joint
//   group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
//   // // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//   // int n = group_variable_values.size();
//   // for(int i = 0; i < n; i++)
//   // {
//   //     ROS_INFO("%d th joint: %f", i, group_variable_values[i]);
//   // }

//   // (80.0, -15.5, 0.0, -0.6, 1.15, -1.15， 41.02, 0.0)
//   group_variable_values[0] = 80;  
//   group_variable_values[1] = -15.5; 
//   group_variable_values[2] = 0.0; 
// /*  group_variable_values[3] = -0.6; 
//   group_variable_values[4] = 1.15; 
//   group_variable_values[5] = -1.15; 
//   group_variable_values[6] = 41.02; 
//   group_variable_values[7] = 0.0; */
//   group.setJointValueTarget(group_variable_values);
//   //group.setRandomTarget();
//   success = group.plan(my_plan);

//   ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

// END_TUTORIAL

  ros::shutdown();  
  return 0;
}

