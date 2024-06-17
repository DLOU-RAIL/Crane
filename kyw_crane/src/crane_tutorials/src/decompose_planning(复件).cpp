#include <ros/ros.h> 
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> 
#include <fstream> 
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

//#include "crane_location_regions.h"
#include "msl/prm.h"
#include "msl/problem.h"
#include <visualization_msgs/Marker.h>
#include "msl/BiMRRTs.h"
#include "construct_robot_trajectory.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

const double PI = 3.1415926;

void split(const std::string& src, const std::string& separator, std::vector<std::string>& dest) //字符串分割到数组
{
    //参数1：要分割的字符串；参数2：作为分隔符的字符；参数3：存放分割后的字符串的vector向量
    std::string str = src;
    std::string substring;
    std::string::size_type start = 0, index;
    dest.clear();
    index = str.find_first_of(separator,start);
    do
    {
        if (index != std::string::npos)
        {
            substring = str.substr(start,index-start );
            dest.push_back(substring);
            start =index+separator.size();
            index = str.find(separator,start);
            if (start == std::string::npos) break;
        }
    }while(index != std::string::npos);

    //the last part
    substring = str.substr(start);
    dest.push_back(substring);
}

class MotionPlanningForCrane
{
public:
  MotionPlanningForCrane()
  {
    init();
  }
  ~MotionPlanningForCrane();
  void init();
  bool planBySamplingSingleConfig(std::string plannerName);
  bool planBySamplingSingleConfig(std::string plannerName, robot_state::RobotState& init_rs, robot_state::RobotState& goal_rs);
  void samplingSingleConfigTest(const std::vector<std::string>& planners, int nRuns);

  bool planByDecomposing(std::string basePlannerName, std::string upperPlannerName);
  bool samplingValidInitGoalConf(CraneLocationRegions& clr, robot_state::RobotState& init_rs, robot_state::RobotState& goal_rs);
  void decomposingTest(const std::vector<std::string>& basePlanners, const std::vector<std::string>& upperPlanners, int nRuns);

  bool isBaseWithDefaultUpperValid(const std::vector<double>& baseConf, robot_state::RobotState& rs);
  bool isCraneConfigValid(const std::vector<double>& craneConf, robot_state::RobotState& rs);
  bool planBasedPRM(std::string upperPlannerName, double dAllowedPlanningTime);

  // 用于生成训练数据的相关函数
  void prepareGenerateData(const std::string &obsFileName);
  void generateLocQualityMap(const std::string &fileName, bool forInit);
  void computeCellQualityForInitCLR(unsigned int i, unsigned int j, unsigned int N);
  void computeCellQualityForGoalCLR(unsigned int i, unsigned int j, unsigned int N);
  void generateValidConfs(std::string file_name)
  {
      int i = 0, j = 0;
      double lift_obj_pos[15*3] = {-33.0, 15.0, 20.0, -38.0, -9.0, 25.0, -4.0, 45.0, 20.0, -4.0, 30.0, 5.0,
                               -4.0, -7.0, 5.0, -14.0, -47.0, 10.0, -60.0, 98.0, 25.0, -60.0, 80.0, 10.0,
                               -40.0, 70.0, 10.0, -45.0, 45.0, 10.0, -45.0, 14.0, 10.0, -30.0, -47.0, 10.0,
                               -30.0, -42.0, 10.0, -41.0, -16.0, 5.0, -18.0, 8.0, 12.0};
      geometry_msgs::Pose pose[15];

      for(i = 0; i < 15; i++)
      {
          pose[i].position.x = lift_obj_pos[i*3];
          pose[i].position.y = lift_obj_pos[i*3+1];
          pose[i].position.z = lift_obj_pos[i*3+2];
          pose[i].orientation.w = 1;
      }

      robot_state::RobotState rs(*rs_);
      int nMaxSamples = 8000;
      int nValidSamples = 0;

      std::vector<geometry_msgs::Point> vis_points;
      geometry_msgs::Point pt;

      // 存储起吊/就位CLR表示
      std::ofstream fout("/home/lys/PycharmProjects/LiftSceneRepresentation/LiftSceneData/collision_free_confs.txt");
//      for(i = 0; i < nX_; i++) {
//          for (j = 0; j < nY_; j++) {
//              fout0 << init_loc_grid_[i][j] << " ";
//              fout1 << goal_loc_grid_[i][j] << " ";
//          }
//          fout0 << std::endl;
//          fout1 << std::endl;
//      }


      for(i = 0; i < 15; i++) {
          ROS_INFO("i: %d", i);
          nValidSamples = 0;
          j = 0;
//          vis_points.clear();
//          visual_tools_->deleteAllMarkers();
          while (j < nMaxSamples) {
              //ROS_INFO("j: %d", j);
              // 求逆解
              bool bIK = rs.setFromIK(joint_model_group_, pose[i], 2, 0.5);

              // 判断是否碰撞
              if (bIK && !scene_->isStateColliding(rs, "whole")) {
                  nValidSamples += 1;
                  double *joint_pos = rs.getVariablePositions();   // 0, 1, 2, 5, 6, 8, 9对应起重机位形
                  // 输出无碰撞位形
                  for(int k = 0; k < 10; k++)
                  {
                      if( k != 3 && k != 4 && k != 7 )
                          fout << joint_pos[k] << ",";
                  }
                  // 输出被吊物位置
                  fout << pose[i].position.x << "," << pose[i].position.y << "," << pose[i].position.z << std::endl;

//                  pt.x = joint_pos[0];
//                  pt.y = joint_pos[1];
//                  pt.z = 0.0;
//                  vis_points.push_back(pt);
//                  visual_tools_->publishSpheres(vis_points, rviz_visual_tools::colors::RED, 0.5, "confs");
//                  visual_tools_->trigger();
              }
              j++;

          }
          //sleep(2);
          ROS_INFO("ValidSamples: %d", nValidSamples);
      }
  }
protected:
  bool generateValidStateByIK(const geometry_msgs::Pose& pose, planning_scene::PlanningScenePtr scene, robot_state::RobotState& rs);
  double computePathLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan, double* dJointWeights);

//protected:
public:
  // 被吊物的起始位姿及终止位姿，只用了4个自由度
  geometry_msgs::Pose initPose_, goalPose_;
  moveit::planning_interface::MoveGroupInterface* whole_group_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr scene_;
  std::string resultPath_;
  robot_state::RobotStatePtr rs_;
  robot_trajectory::RobotTrajectoryPtr robot_traj_;

    // 生成训练数据相关变量
    // 吊装场景的中心
    double scene_org_x_, scene_org_y_;
    // 整个吊装场景的长和宽
    double scene_size_x_, scene_size_y_;
    // X、Y方向栅格数量
    unsigned int nX_, nY_;
    unsigned int **obs_;
    // 栅格化后起重机器人的潜在站位栅格
    unsigned int **init_loc_grid_;
    unsigned int **goal_loc_grid_;
    // 站位质量图
    unsigned int **init_loc_quality_map_;
    unsigned int **goal_loc_quality_map_;

    mutable random_numbers::RandomNumberGenerator rng_;
    CraneLocationRegions *init_clr_, *goal_clr_;

  // 调试用
  ros::Publisher jointPub_;
  moveit_msgs::RobotState rs_msg_;
  ros::Publisher display_publisher_;
  moveit_msgs::DisplayTrajectory display_trajectory_;
  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  visualization_msgs::Marker markerVertexes_;

  // 统计时间
  timeval timer_;
};

class Vector2
{
public:
    double x, y;
    Vector2(double dX = 0.0, double dY = 0.0)
    {
        x = dX;
        y = dY;
    }
    ~Vector2(){};
    static Vector2 Max(Vector2 &v0, Vector2 &v1)
    {
        return Vector2(max(v0.x, v0.y), max(v1.x, v1.y));
    }
    double SqrMagnitude()
    {
        return x*x + y*y;
    }
};

bool isInterSection(double x_min, double x_max, double y_min, double y_max, double org_x, double org_y, double r, double R)
{
    double center_x = 0.5 * (x_min + x_max);
    double center_y = 0.5 * (y_min + y_max);
    double sqr_dist = (center_x - org_x) * (center_x - org_x) + (center_y - org_y) * (center_y - org_y);
    return  r * r < sqr_dist && sqr_dist < R * R;
//    Vector2 v0(0.5*(x_min+x_max)-org_x, 0.5*(y_min+y_max)-org_y);
//    Vector2 v1(-v0.x, -v0.y);
//    Vector2 v = Vector2::Max(v0, v1);
//    Vector2 w(v.x-0.5*(x_max-x_min), v.y-0.5*(y_max-y_min));
//    Vector2 zero(0,0);
//    Vector2 u = Vector2::Max(w, zero);
//    return u.SqrMagnitude() < r*r;
}

void MotionPlanningForCrane::computeCellQualityForInitCLR(unsigned int i, unsigned int j, unsigned int N)
{ROS_INFO("In computeCellQualityForInitCLR");
    std::vector<double> base_conf(3, 0.0), crane_conf(8, 0.0);
    robot_state::RobotState init_rs(*whole_group_->getCurrentState()), goal_rs(*whole_group_->getCurrentState());
    double step_x = scene_size_x_ / nX_;
    double step_y = scene_size_y_ / nY_;

    if(!init_loc_grid_[i][j])
        return;     // 略过ij单元格

    for (int k = 0; k < N; k++) {
        base_conf[0] = rng_.uniformReal(i * step_x, (i + 1) * step_x);
        base_conf[1] = rng_.uniformReal(j * step_y, (j + 1) * step_y);
        base_conf[0] += -0.5 * scene_size_x_ + scene_org_x_;
        base_conf[1] += -0.5 * scene_size_y_ + scene_org_y_;
        base_conf[2] = rng_.uniformReal(-PI, PI);

        init_clr_->upperIK(base_conf, crane_conf);
        for(int q = 0; q < 8; q++)
            printf("%f  ", crane_conf[q]);
        printf("\n");
        if( isCraneConfigValid(crane_conf, init_rs) )
        {printf("dfshfddddddddddddddddd\n");
            if( generateValidStateByIK(goalPose_, scene_, goal_rs) )
            {printf("hhhhhhhhhhhhhhhhhhhhhhhh\n");
                if( planBySamplingSingleConfig("RRTConnectkConfigDefault", init_rs, goal_rs) )
                {
                    init_loc_quality_map_[i][j] += 1;
                }
            }
        }
    }
}

void MotionPlanningForCrane::computeCellQualityForGoalCLR(unsigned int i, unsigned int j, unsigned int N)
{
    std::vector<double> base_conf(3, 0.0), crane_conf(8, 0.0);
    robot_state::RobotState init_rs(*whole_group_->getCurrentState()), goal_rs(*whole_group_->getCurrentState());
    double step_x = scene_size_x_ / nX_;
    double step_y = scene_size_y_ / nY_;

    if(!goal_loc_grid_[i][j])
        return;     // 略过ij单元格

    ROS_INFO("In computeCellQualityForGoalCLR");
    ROS_INFO("Calculating Grid[%d][%d]", i, j);

    for (int k = 0; k < N; k++) {
        base_conf[0] = rng_.uniformReal(i * step_x, (i + 1) * step_x);
        base_conf[1] = rng_.uniformReal(j * step_y, (j + 1) * step_y);
        base_conf[0] += -0.5 * scene_size_x_ + scene_org_x_;
        base_conf[1] += -0.5 * scene_size_y_ + scene_org_y_;
        base_conf[2] = rng_.uniformReal(-PI, PI);

        goal_clr_->upperIK(base_conf, crane_conf);
        if( isCraneConfigValid(crane_conf, goal_rs) )
        {
            if( generateValidStateByIK(initPose_, scene_, init_rs) )
            {
                if( planBySamplingSingleConfig("RRTConnectkConfigDefault", init_rs, goal_rs) )
                {
                    goal_loc_quality_map_[i][j] += 1;
                }
            }
        }
    }
}

void MotionPlanningForCrane::prepareGenerateData(const std::string &obsFileName)
{
    scene_org_x_ = 0.0;
    scene_org_y_ = 0.0;
    scene_size_x_ = 250.0;
    scene_size_y_ = 250.0;
    nX_ = 250;
    nY_ = 250;
    int i=0, j=0;
    init_clr_ = new CraneLocationRegions(initPose_, 10.0, 28.0);
    goal_clr_ = new CraneLocationRegions(goalPose_, 10.0, 28.0);


    // 申请内存空间
    obs_ = new unsigned int* [nX_];
    init_loc_grid_ = new unsigned int* [nX_];
    goal_loc_grid_ = new unsigned int* [nX_];
    init_loc_quality_map_ = new unsigned int* [nX_];
    goal_loc_quality_map_ = new unsigned int* [nX_];
    for(i = 0; i < nX_; i++)
    {
        obs_[i] = new unsigned int [nY_];
        init_loc_grid_[i] = new unsigned int [nY_];
        goal_loc_grid_[i] = new unsigned int [nY_];
        init_loc_quality_map_[i] = new unsigned int [nY_];
        goal_loc_quality_map_[i] = new unsigned int [nY_];
    }

    // 加载障碍物地图
    i = 0, j = 0;
    std::ifstream fin(obsFileName);
    if(fin) {
        std::string str;
        std::vector<std::string> data;
        while (getline(fin, str)) {
            //ROS_INFO("%s", str.c_str());
            split(str, " ", data);
            for(j = 0; j < nY_; j++)
            {
                obs_[i][j] = atoi(data[j].c_str());
            }
            i++;
        }
    }

//    // 站位地图
//    std::ifstream fin1("/home/lys/PycharmProjects/LiftSceneRepresentation/init_loc_grid.txt");
//    std::ifstream fin2("/home/lys/PycharmProjects/LiftSceneRepresentation/goal_loc_grid.txt");
//    i = 0, j = 0;
//    if(fin1) {
//        std::string str;
//        std::vector<std::string> data;
//        while (getline(fin1, str)) {
//            //ROS_INFO("%s", str.c_str());
//            split(str, " ", data);
//            for(j = 0; j < nY_; j++)
//            {
//                init_loc_grid_[i][j] = atoi(data[j].c_str());
//            }
//            i++;
//        }
//    }
//    i = 0, j = 0;
//    if(fin2) {
//        std::string str;
//        std::vector<std::string> data;
//        while (getline(fin2, str)) {
//            //ROS_INFO("%s", str.c_str());
//            split(str, " ", data);
//            for(j = 0; j < nY_; j++)
//            {
//                goal_loc_grid_[i][j] = atoi(data[j].c_str());
//            }
//            i++;
//        }
//    }

    //ROS_INFO("INIT_CLR: %.2f, %.2f", init_clr_->workingRadiusMin_, init_clr_->workingRadiusMax_);

    // 生成起重机器人潜在站位栅格
    double x_min, x_max, y_min, y_max;
    double step_x = scene_size_x_ / nX_;
    double step_y = scene_size_y_ / nY_;
    for(i = 0; i < nX_; i++)
    {
        for(j = 0; j < nY_; j++)
        {
            x_min = i*step_x -0.5 * scene_size_x_ + scene_org_x_;
            x_max = (i+1)*step_x -0.5 * scene_size_x_ + scene_org_x_;
            y_min = j * step_y -0.5 * scene_size_y_ + scene_org_y_;
            y_max = (j+1) * step_y -0.5 * scene_size_y_ + scene_org_y_;
            init_loc_grid_[i][j] = isInterSection(x_min, x_max, y_min, y_max, initPose_.position.x, initPose_.position.y,
                    init_clr_->workingRadiusMin_, init_clr_->workingRadiusMax_);
            goal_loc_grid_[i][j] = isInterSection(x_min, x_max, y_min, y_max, goalPose_.position.x, goalPose_.position.y,
                    goal_clr_->workingRadiusMin_, goal_clr_->workingRadiusMax_);

            if(obs_[i][j])
            {
                geometry_msgs::Point vertex_;
                vertex_.x = 0.5 * (x_min + x_max);
                vertex_.y = 0.5 * (y_min + y_max);
                markerVertexes_.points.push_back(vertex_);
            }
//            if(init_loc_grid_[i][j])
//            {
//                geometry_msgs::Point vertex_;
//                vertex_.x = 0.5 * (x_min + x_max);
//                vertex_.y = 0.5 * (y_min + y_max);
//                markerVertexes_.points.push_back(vertex_);
//            }
//            if(goal_loc_grid_[i][j])
//            {
//                geometry_msgs::Point vertex_;
//                vertex_.x = 0.5 * (x_min + x_max);
//                vertex_.y = 0.5 * (y_min + y_max);
//                markerVertexes_.points.push_back(vertex_);
//            }
        }
    }

//    // 存储起吊/就位CLR表示
//    std::ofstream fout0("/home/lys/PycharmProjects/LiftSceneRepresentation/init_loc_grid.txt");
//    std::ofstream fout1("/home/lys/PycharmProjects/LiftSceneRepresentation/goal_loc_grid.txt");
//    for(i = 0; i < nX_; i++) {
//        for (j = 0; j < nY_; j++) {
//            fout0 << init_loc_grid_[i][j] << " ";
//            fout1 << goal_loc_grid_[i][j] << " ";
//        }
//        fout0 << std::endl;
//        fout1 << std::endl;
//    }
//
//    // 初始化站位质量图
//    for(int i = 0; i < nX_; i++)
//    {
//        for(int j = 0; j < nY_; j++)
//        {
//            init_loc_quality_map_[i][j] = goal_loc_quality_map_[i][j] = 0;
//        }
//    }

    ROS_INFO("Before Publish!!!");

//    while(1)
//    {
//        visual_tools_->publishMarker(markerVertexes_);
//        visual_tools_->trigger();
//    }

}

void MotionPlanningForCrane::generateLocQualityMap(const std::string &fileName, bool forInit) {
    ROS_INFO("In Gen Quality Map");
    for(int i = 0; i < nX_; i++)
    {
        for(int j = 0; j < nY_; j++)
        {
            if(forInit) {
                if(!obs_[i][j] && init_loc_grid_[i][j])
                {
                    computeCellQualityForInitCLR(i, j, 30);
                }
            }
            else
            {
                if(!obs_[i][j] && goal_loc_grid_[i][j])
                {
                    computeCellQualityForGoalCLR(i, j, 30);
                }
            }
        }
    }

    // 存储起吊/就位CLR质量图
    int i, j;
    std::ofstream fout(fileName);
    for(i = 0; i < nX_; i++) {
        for (j = 0; j < nY_; j++) {
            if(forInit)
            {
                fout << init_loc_quality_map_[i][j] << " ";
            }
            else{
                fout << goal_loc_quality_map_[i][j] << " ";
            }


        }
        fout << std::endl;
    }
}

MotionPlanningForCrane::~MotionPlanningForCrane()
{
  if(whole_group_)
    delete whole_group_;
}
#include <moveit/robot_model/joint_model.h>
void MotionPlanningForCrane::init()
{
  // 设置被吊物起始位姿和终止位姿
  /*initPose_.position.x = -26.3;
  initPose_.position.y = -47.20;
  initPose_.position.z = 7.0;
  initPose_.orientation.w = 1;
  goalPose_.position.x = 99.85;
  goalPose_.position.y = -31.9;
  goalPose_.position.z = 9.52;
  goalPose_.orientation.w = 1;*/

  initPose_.position.x = 8.0;
  initPose_.position.y = 110.0;
  initPose_.position.z = 10.0;
  initPose_.orientation.w = 1;
  goalPose_.position.x = -18.0;
  goalPose_.position.y = 8.0;
  goalPose_.position.z = 12;
  goalPose_.orientation.w = 1;


  // 设置规划组及相关配置工作
  whole_group_ = new moveit::planning_interface::MoveGroupInterface("whole");
  robot_state::RobotState robot_state_ = robot_state::RobotState(*whole_group_->getCurrentState());
  joint_model_group_ = robot_state_.getJointModelGroup("whole");

  
  // std::vector<const robot_model::JointModel::Bounds*> limits = joint_model_group_->getActiveJointModelsBounds();
  // for(int i = 0; i < limits.size(); i++)
  // {
  //   ROS_ERROR("%d: [%f, %f]", i, limits[i]->at(0).min_position_, limits[i]->at(0).max_position_);
  // }
  // joint_model_group_ = robot_state_.getJointModelGroup("upper");
  // int n = joint_model_group_->getVariableCount() - joint_model_group_->getMimicJointModels().size();
  // ROS_ERROR("NUM = %d", n);
  // joint_model_group_ = robot_state_.getJointModelGroup("base");
  // n = joint_model_group_->getVariableCount()- joint_model_group_->getMimicJointModels().size();
  // ROS_ERROR("NUM = %d", n);

  // 获得当前活动的场景，为碰撞检测做好准备
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef;
  monitor_ptr_udef.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));  
  monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");  
  planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);  
  ps->getCurrentStateNonConst().update();  
  scene_ = ps->diff();  
  scene_->decoupleParent();

  // 设置结果存储路径
  resultPath_ = ros::package::getPath("crane_tutorials") + std::string("/motion_planning_results");
  rs_ = whole_group_->getCurrentState();
  // 调试用
  ros::NodeHandle node_handle; 
  jointPub_ = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1); 
  display_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom","/moveit_visual_markers"));
  visual_tools_->deleteAllMarkers();
    markerVertexes_.header.frame_id  = "odom";
    markerVertexes_.header.stamp = ros::Time::now();
    markerVertexes_.action = visualization_msgs::Marker::ADD;
    markerVertexes_.pose.orientation.w = 1.0;
    markerVertexes_.id = 0;
    markerVertexes_.type = visualization_msgs::Marker::POINTS;
    markerVertexes_.scale.x = 0.5;
    markerVertexes_.scale.y = 0.5;
    markerVertexes_.color.g = 1.0f;
    markerVertexes_.color.a = 1.0;
    ROS_INFO("In Init!!");
}

bool MotionPlanningForCrane::planBySamplingSingleConfig(std::string plannerName)
{
  double dStartTime, dCurrentTime, dSamplingInitConfTime, dSamplingGoalConfTime, dPlanningTime;
  bool bInitFound, bGoalFound, bPathFound;
  bInitFound = bGoalFound = bPathFound = false;

  // 起重机器人起始/终止状态采样
  robot_state::RobotState init_rs(*whole_group_->getCurrentState());
  robot_state::RobotState goal_rs(*whole_group_->getCurrentState());
  gettimeofday(&timer_, NULL);
  dStartTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  bInitFound = generateValidStateByIK(initPose_, scene_, init_rs);
  if(bInitFound)
    ROS_INFO("Initial Robot State Found!!");
  else
    ROS_INFO("Initial Robot State NOT Found!!");

  gettimeofday(&timer_, NULL);
  dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  dSamplingInitConfTime = dCurrentTime - dStartTime;

  dStartTime = dCurrentTime;
  bGoalFound = generateValidStateByIK(goalPose_, scene_, goal_rs);
  if(bGoalFound)
    ROS_INFO("Goal Robot State Found!!");
  else
    ROS_INFO("Goal Robot State NOT Found!!");
  gettimeofday(&timer_, NULL);
  dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  dSamplingGoalConfTime = dCurrentTime - dStartTime;

  // 设置规划任务
  moveit::planning_interface::MoveGroupInterface::Plan whole_plan;   // 其中的joint_trajectory只包含所规划的几个自由度，不包括mimic joint
  double dMaxPlanningTime = 120;
  // if(bInitFound && bGoalFound)
  // {
    // 设置工作空间范围
    whole_group_->setWorkspace(-80, -130, -1, 80, 130, 100);
    whole_group_->setPlannerId(plannerName);
    whole_group_->setPlanningTime(dMaxPlanningTime);
    whole_group_->setNumPlanningAttempts(2);
    // 设置起始位形
    whole_group_->setStartState(init_rs);
    // 设置目标位形
    whole_group_->setJointValueTarget(goal_rs);
    //whole_group_->setPoseTarget(goalPose_);  // 无法规划，规划失败！！！
    // 求解规划问题
    bPathFound = whole_group_->plan(whole_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(bPathFound)
    {
      ROS_INFO("FOUND A PATH FOR CRANE!!");
      dPlanningTime = whole_plan.planning_time_;
    }
    else
    {
      ROS_INFO("FAILURE!!");
      dPlanningTime = dMaxPlanningTime;
    }
  // }
  // else
  // {
  //   dPlanningTime = 0.0;
  // }

  // 对结果进行后处理，记录：起吊位形，就位位形，起始位形采样时间，终止位形采样时间，规划是否成功，规划时间，路径长度
  // world_joint/x, world_joint/y, world_joint/theta, chassis_to_wheel_left_joint, chassis_to_wheel_right_joint,
  // superStructure_joint, boom_joint, rope_joint, hook_block_joint, hook_joint
  double *init_pos = init_rs.getVariablePositions();
  double *goal_pos = goal_rs.getVariablePositions();
  double dJointWeights[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double dPathLen;
  if(bPathFound)
    dPathLen = computePathLength(whole_plan, dJointWeights);
  else
    dPathLen = 10e-40;
  std::string strFileName = resultPath_ + "/" + plannerName + ".csv";
	std::ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
  static bool bFirstWrite = true;
	if( bFirstWrite )
	{
		fout << "起吊位形x" << ",起吊位形y" << ",起吊位形alpha" << ",起吊位形beta" << ",起吊位形gama" << ",起吊位形h" << ",起吊位形w"
		     << ",就位位形x" << ",就位位形y" << ",就位位形alpha" << ",就位位形beta" << ",就位位形gama" << ",就位位形h" << ",就位位形w"
		     << ",找到路径" << ",路径长度" << ",总规划时间" << ",起始位形采样成功" << ",起始位形采样时间"
		     << ",目标位形采样成功" << ",目标位形采样时间" << ",规划算法运行时间";
		bFirstWrite = false;
	}
	fout.seekp( 0, std::ios::end );

	fout << "\n" << init_pos[0] << ","  << init_pos[1] << ","  << init_pos[2] << ","  << init_pos[5] << ","  << init_pos[6] << ","  << init_pos[8] << ","  << init_pos[9] << ","
            << goal_pos[0] << ","  << goal_pos[1] << ","  << goal_pos[2] << ","  << goal_pos[5] << ","  << goal_pos[6] << ","  << goal_pos[8] << ","  << goal_pos[9] << ","
            << bPathFound << "," << dPathLen << "," << dSamplingInitConfTime + dSamplingGoalConfTime + dPlanningTime << ","
         << bInitFound << "," << dSamplingInitConfTime << "," << bGoalFound << "," << dSamplingGoalConfTime << "," << dPlanningTime;
	fout.close();

	return bPathFound;
}

bool MotionPlanningForCrane::planBySamplingSingleConfig(std::string plannerName, robot_state::RobotState& init_rs, robot_state::RobotState& goal_rs)
{
    double dStartTime, dCurrentTime, dSamplingInitConfTime, dSamplingGoalConfTime, dPlanningTime;
    bool bInitFound, bGoalFound, bPathFound;
    bInitFound = bGoalFound = true;
    bPathFound = false;

    gettimeofday(&timer_, NULL);
    dStartTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);

    dSamplingInitConfTime = 0.0;
    dSamplingGoalConfTime = 0.0;

    // 设置规划任务
    moveit::planning_interface::MoveGroupInterface::Plan whole_plan;   // 其中的joint_trajectory只包含所规划的几个自由度，不包括mimic joint
    double dMaxPlanningTime = 120;

    // 设置工作空间范围
    whole_group_->setWorkspace(-120, -130, -1, 120, 130, 100);
    whole_group_->setPlannerId(plannerName);
    whole_group_->setPlanningTime(dMaxPlanningTime);
    whole_group_->setNumPlanningAttempts(2);
    // 设置起始位形
    whole_group_->setStartState(init_rs);
    // 设置目标位形
    whole_group_->setJointValueTarget(goal_rs);
    //whole_group_->setPoseTarget(goalPose_);  // 无法规划，规划失败！！！
    // 求解规划问题
    bPathFound = whole_group_->plan(whole_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(bPathFound)
    {
        ROS_INFO("FOUND A PATH FOR CRANE!!");
        dPlanningTime = whole_plan.planning_time_;
    }
    else
    {
        ROS_INFO("FAILURE!!");
        dPlanningTime = dMaxPlanningTime;
    }

    // 对结果进行后处理，记录：起吊位形，就位位形，起始位形采样时间，终止位形采样时间，规划是否成功，规划时间，路径长度
    // world_joint/x, world_joint/y, world_joint/theta, chassis_to_wheel_left_joint, chassis_to_wheel_right_joint,
    // superStructure_joint, boom_joint, rope_joint, hook_block_joint, hook_joint
    double *init_pos = init_rs.getVariablePositions();
    double *goal_pos = goal_rs.getVariablePositions();
    double dJointWeights[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double dPathLen;
    if(bPathFound)
        dPathLen = computePathLength(whole_plan, dJointWeights);
    else
        dPathLen = 10e-40;
    std::string strFileName = resultPath_ + "/" + plannerName + ".csv";
    std::ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
    static bool bFirstWrite = true;
    if( bFirstWrite )
    {
        fout << "起吊位形x" << ",起吊位形y" << ",起吊位形alpha" << ",起吊位形beta" << ",起吊位形gama" << ",起吊位形h" << ",起吊位形w"
             << ",就位位形x" << ",就位位形y" << ",就位位形alpha" << ",就位位形beta" << ",就位位形gama" << ",就位位形h" << ",就位位形w"
             << ",找到路径" << ",路径长度" << ",总规划时间" << ",起始位形采样成功" << ",起始位形采样时间"
             << ",目标位形采样成功" << ",目标位形采样时间" << ",规划算法运行时间";
        bFirstWrite = false;
    }
    fout.seekp( 0, std::ios::end );

    fout << "\n" << init_pos[0] << ","  << init_pos[1] << ","  << init_pos[2] << ","  << init_pos[5] << ","  << init_pos[6] << ","  << init_pos[8] << ","  << init_pos[9] << ","
         << goal_pos[0] << ","  << goal_pos[1] << ","  << goal_pos[2] << ","  << goal_pos[5] << ","  << goal_pos[6] << ","  << goal_pos[8] << ","  << goal_pos[9] << ","
         << bPathFound << "," << dPathLen << "," << dSamplingInitConfTime + dSamplingGoalConfTime + dPlanningTime << ","
         << bInitFound << "," << dSamplingInitConfTime << "," << bGoalFound << "," << dSamplingGoalConfTime << "," << dPlanningTime;
    fout.close();

    return bPathFound;
}

void MotionPlanningForCrane::samplingSingleConfigTest(const std::vector<std::string>& planners, int nRuns)
{
  int nPlanners = planners.size();
  for(int i = 0; i < nPlanners; i++)
  {
    ROS_INFO("Testing %s:", planners[i].c_str());
    for(int j = 0; j < nRuns; j++)
    {
      ROS_INFO("The %d th running", j);
      planBySamplingSingleConfig(planners[i]);
    }
  }
}

bool MotionPlanningForCrane::samplingValidInitGoalConf(CraneLocationRegions& clr, robot_state::RobotState& init_rs, robot_state::RobotState& goal_rs)
{
  bool bInitFound, bGoalFound;
  bInitFound = bGoalFound = false;
  robot_state::RobotState rs(*whole_group_->getCurrentState());   // 这是耗时操作，可将其放到构造函数
  std::vector<double> baseConf(3, 0.0);
  // 设置起重机器人行走过程中最安全的上车位形
  std::vector<double> craneConf(8, 0.0);
  craneConf[4] = 1.45;
  craneConf[5] = -1.45;
  craneConf[6] = 50;
  rs.setJointGroupPositions(joint_model_group_, craneConf);
  // 用最安全的上车位形来在站位扇环中进行采样，获得无碰撞的起始和目标位形
  while(ros::ok())
  {
    clr.samplingBaseConfig(baseConf);
    craneConf[0] = baseConf[0];
    craneConf[1] = baseConf[1];
    craneConf[2] = baseConf[2];
    craneConf[3] = 0.0;
    craneConf[4] = 1.45;
    craneConf[5] = -1.45;
    craneConf[6] = 50;
    craneConf[7] = 0.0;
    rs.setJointGroupPositions(joint_model_group_, craneConf);
    // // 同步到rviz中，用于调试
    // robot_state::robotStateToRobotStateMsg(rs, rs_msg_);
    // jointPub_.publish(rs_msg_.joint_state);
    if(!scene_->isStateColliding(rs, "whole"))
    {
      goal_rs = rs;
      bGoalFound = true;
      ROS_INFO("Goal Collision Free");
    }
    else
    {
      bGoalFound = false;
      ROS_ERROR("COLLISION");
    }
    //sleep(5);

    if(bGoalFound)
    {
      // 根据pick_goal_rs求解pick_init_rs
      clr.upperIK(baseConf, craneConf);
      rs.setJointGroupPositions(joint_model_group_, craneConf);
      // // 同步到rviz中，用于调试
      // robot_state::robotStateToRobotStateMsg(rs, rs_msg_);
      // jointPub_.publish(rs_msg_.joint_state);
      if(!scene_->isStateColliding(rs, "whole"))
      {
        init_rs = rs;
        bInitFound = true;
        ROS_INFO("Initial Collision Free");
        break;
      }
      else
      {
        bInitFound = false;
        ROS_ERROR("COLLISION");
      }
      //sleep(5);
    }
    ROS_INFO("\n");
  }

  return (bInitFound && bGoalFound);
}


bool MotionPlanningForCrane::planByDecomposing(std::string basePlannerName, std::string upperPlannerName)
{
  double dStartTime, dCurrentTime, dSamplingInitConfTime, dSamplingGoalConfTime, dPlanningTime;
  bool bInitFound, bGoalFound, bPathFound;
  bInitFound = bGoalFound = bPathFound = false;

  double dPickPlanningTime, dPlacePlanningTime, dMovePlanningTime;
  bool bPickSuccess, bPlaceSuccess, bMoveSuccess;
  bPickSuccess = bPlaceSuccess = bMoveSuccess = false;
  double dMaxPlanningTime = 10.0;

  // PICK PLANNING
  moveit::planning_interface::MoveGroupInterface upper_group("upper");
  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
  // 起重机器人起始/终止状态采样
  robot_state::RobotState pick_init_rs(*whole_group_->getCurrentState());   // 非常耗时的操作，慎用
  robot_state::RobotState pick_goal_rs(*whole_group_->getCurrentState());
  CraneLocationRegions pickCLR(initPose_);
  gettimeofday(&timer_, NULL);
  dStartTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  bool bSamplingSuccess = samplingValidInitGoalConf(pickCLR, pick_init_rs, pick_goal_rs);
  gettimeofday(&timer_, NULL);
  dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  dSamplingInitConfTime = dCurrentTime - dStartTime;
  if( bSamplingSuccess )
  {
    bInitFound = true;
    // 起吊规划
    dMaxPlanningTime = 10;
    upper_group.setPlannerId(upperPlannerName);
    upper_group.setPlanningTime(dMaxPlanningTime);
    upper_group.setNumPlanningAttempts(2);
    // 设置起始位形
    upper_group.setStartState(pick_init_rs);
    // 设置目标位形
    upper_group.setJointValueTarget(pick_goal_rs);
    // 求解规划问题
    bPickSuccess = upper_group.plan(pick_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(bPickSuccess)
    {
      ROS_INFO("PICKING SUCCESS!!");
      dPickPlanningTime = pick_plan.planning_time_;
    }
    else
    {
      ROS_INFO("FAILURE!!");
      dPickPlanningTime = dMaxPlanningTime;
    }
  }


  // PLACE PLANNING
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;
  // 起重机器人起始/终止状态采样
  robot_state::RobotState place_init_rs(*whole_group_->getCurrentState());
  robot_state::RobotState place_goal_rs(*whole_group_->getCurrentState());
  CraneLocationRegions placeCLR(goalPose_);
  gettimeofday(&timer_, NULL);
  dStartTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  bSamplingSuccess = samplingValidInitGoalConf(placeCLR, place_goal_rs, place_init_rs);
  gettimeofday(&timer_, NULL);
  dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  dSamplingGoalConfTime = dCurrentTime - dStartTime;
  if( bSamplingSuccess )
  {
    bGoalFound = true;
    // 起吊规划
    dMaxPlanningTime = 10;
    upper_group.setPlannerId(upperPlannerName);
    upper_group.setPlanningTime(dMaxPlanningTime);
    upper_group.setNumPlanningAttempts(2);
    // 设置起始位形
    upper_group.setStartState(place_init_rs);
    // 设置目标位形
    upper_group.setJointValueTarget(place_goal_rs);
    // 求解规划问题
    bPlaceSuccess = upper_group.plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(bPlaceSuccess)
    {
      ROS_INFO("PLACING SUCCESS!!");
      dPlacePlanningTime = place_plan.planning_time_;
    }
    else
    {
      ROS_INFO("FAILURE!!");
      dPlacePlanningTime = dMaxPlanningTime;
    }
  }

  // MOVE PLANNING
  moveit::planning_interface::MoveGroupInterface base_group("base");
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;
  dMaxPlanningTime = 80;
  // 设置工作空间范围
    base_group.setWorkspace(-60, -50, -1, 150, 40, 100);
    base_group.setPlannerId(basePlannerName);
    base_group.setPlanningTime(dMaxPlanningTime);
    base_group.setNumPlanningAttempts(2);
    // 设置起始位形
    base_group.setStartState(pick_goal_rs);
    // 设置目标位形
    base_group.setJointValueTarget(place_init_rs);
    // 求解规划问题
    bMoveSuccess = base_group.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(bMoveSuccess)
    {
      ROS_INFO("MOVE SUCCESS!!");
      dMovePlanningTime = move_plan.planning_time_;
    }
    else
    {
      ROS_INFO("FAILURE!!");
      dMovePlanningTime = dMaxPlanningTime;
    }

    display_trajectory_.trajectory_start = pick_plan.start_state_;
    if(bPickSuccess)
      display_trajectory_.trajectory.push_back(pick_plan.trajectory_);
    if(bMoveSuccess)
      display_trajectory_.trajectory.push_back(move_plan.trajectory_);
    if(bPlaceSuccess)
      display_trajectory_.trajectory.push_back(place_plan.trajectory_);
    display_publisher_.publish(display_trajectory_);


  // 对结果进行后处理，记录：起始位形采样时间，终止位形采样时间，规划是否成功，规划时间，路径长度
  double dBaseJointWeights[3] = {1.0, 1.0, 1.0};
  double dUpperJointWeights[4] = {1.0, 1.0, 1.0, 1.0};
  double dPathLen;
  bPathFound = bPickSuccess && bMoveSuccess && bPlaceSuccess;
  if(bPathFound)
    dPathLen = computePathLength(move_plan, dBaseJointWeights) + computePathLength(pick_plan, dUpperJointWeights) + computePathLength(place_plan, dUpperJointWeights);
  else
    dPathLen = 10e-40;
  std::string strFileName = resultPath_ + "/decompose/B_" + basePlannerName + "+U_" + upperPlannerName + ".csv";
	std::ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
  static bool bFirstWrite = true;
	if( bFirstWrite )
	{
		fout << "找到路径" << ",路径长度" << ",总规划时间" << ",起始位形采样成功" << ",起始位形采样时间" << ",目标位形采样成功" << ",目标位形采样时间" << ",规划算法运行时间"
         << ",起吊规划时间" << ",行走规划时间" << ",就位规划时间";
		bFirstWrite = false;
	}
	fout.seekp( 0, std::ios::end );
  dPlanningTime = dPickPlanningTime + dMovePlanningTime + dPlacePlanningTime;
  double dTotalPlanningTime = dSamplingInitConfTime + dSamplingGoalConfTime + dPlanningTime;
  
	fout << "\n" << bPathFound << "," << dPathLen << "," << dTotalPlanningTime << "," 
       << bInitFound << "," << dSamplingInitConfTime << "," << bGoalFound << "," << dSamplingGoalConfTime << "," << dPlanningTime << ","
       << dPickPlanningTime << "," << dMovePlanningTime << "," << dPlacePlanningTime;
	fout.close();
}

void MotionPlanningForCrane::decomposingTest(const std::vector<std::string>& basePlanners, const std::vector<std::string>& upperPlanners, int nRuns)
{
  int nPlanners = basePlanners.size();
  for(int i = 0; i < nPlanners; i++)
  {
    ROS_INFO("Testing %s + %s :", basePlanners[i].c_str(), upperPlanners[i].c_str());
    for(int j = 0; j < nRuns; j++)
    {
      ROS_INFO("The %d th running", j);
      planByDecomposing(basePlanners[i], upperPlanners[i]);
    }
  }
}

bool MotionPlanningForCrane::isBaseWithDefaultUpperValid(const std::vector<double>& baseConf, robot_state::RobotState& rs)
{
  std::vector<double> craneConf(8, 0.0);
  craneConf[0] = baseConf[0];
  craneConf[1] = baseConf[1];
  craneConf[2] = baseConf[2];
  craneConf[3] = 0.0;
  craneConf[4] = 1.45;
  craneConf[5] = -1.45;
  craneConf[6] = 50;
  craneConf[7] = 0.0;
  rs.setJointGroupPositions(joint_model_group_, craneConf);
	scene_->setCurrentState(rs);
  robot_state::RobotState& current_state = scene_->getCurrentStateNonConst();
	bool bSatisfied = scene_->isStateValid(current_state, "whole");
  return bSatisfied;
}

bool MotionPlanningForCrane::isCraneConfigValid(const std::vector<double>& craneConf, robot_state::RobotState& rs)
{
  rs.setJointGroupPositions(joint_model_group_, craneConf);
	scene_->setCurrentState(rs);
  robot_state::RobotState& current_state = scene_->getCurrentStateNonConst();
	bool bSatisfied = scene_->isStateValid(current_state, "whole");
  return bSatisfied;
}

bool MotionPlanningForCrane::planBasedPRM(std::string upperPlannerName, double dAllowedPlanningTime)
{
  double dRemainingTime = dAllowedPlanningTime;
  gettimeofday(&timer_, NULL);
  double dStartTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  bool bSuccess = false;
  moveit::planning_interface::MoveGroupInterface::Plan pick_plan, place_plan, move_plan;

  // 1.用最安全的上车位姿构建roadmap
  CraneLocationRegions pickCLR(initPose_), placeCLR(goalPose_);
  Problem* pProlem = new Problem("base", &pickCLR, &placeCLR);
  PRM prm(pProlem);
  prm.bVisual_ = true;
  ROS_INFO("KKKKKKKKKKKKKKKKKKKKKKKKK");
    
  // 构建roadmap
  prm.Construct(3000);

  // 2.利用CLR采样站位环内的下车位姿，并尝试规划，若经常失败，则增量继续构建roadmap，直到规划成功
  robot_state::RobotState pick_init_rs(*whole_group_->getCurrentState());
  robot_state::RobotState pick_goal_rs(*whole_group_->getCurrentState());
  robot_state::RobotState place_init_rs(*whole_group_->getCurrentState());
  robot_state::RobotState place_goal_rs(*whole_group_->getCurrentState());
  double dMovePlanningTime = 0;
  gettimeofday(&timer_, NULL);
  double dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  dRemainingTime = dAllowedPlanningTime - (dCurrentTime - dStartTime);
  int nAttempts = 20;  // 尝试规划10次，若均失败则需增加节点
  bool bMoveSuccess = false;
  std::vector<double> craneConf(8, 0.0);
  std::vector<double> v;
  double dMovePathLen = 0;
  while(!bMoveSuccess && dRemainingTime > 0.0)
  {
    if(nAttempts > 0)   // 尝试规划
    {
      nAttempts--;
      while(1)      // 确保：1）起点和终点有效，否则无法进行下车规划；2）对应的起吊位形和就位位形是有效的，否则两端上车规划无法规划。
      {
        pickCLR.samplingBaseConfig(v);
        pickCLR.upperIK(v, craneConf);
        isBaseWithDefaultUpperValid(v, pick_goal_rs);
        if(!isBaseWithDefaultUpperValid(v, pick_goal_rs) || !isCraneConfigValid(craneConf, pick_init_rs))
        {
          continue;
        }
        pProlem->InitialState = MSLVector(v[0], v[1], v[2]);

        placeCLR.samplingBaseConfig(v);
        placeCLR.upperIK(v, craneConf);
        if(!isBaseWithDefaultUpperValid(v, place_init_rs) || !isCraneConfigValid(craneConf, place_goal_rs))
        {
          continue;
        }
        pProlem->GoalState = MSLVector(v[0], v[1], v[2]);
    
        break;
      }

      // 寻找路径
      bMoveSuccess = prm.Plan();
      if(bMoveSuccess)    // 对规划结果进行处理
      {
        // 构造moveit风格的路径
        // 计算路径长度
        list<MSLVector>::iterator it;
        int i = 0;
        it=prm.Path.begin();
        MSLVector prePoint = (*it);
        it++;
        for(; it!=prm.Path.end(); it++)
        {
          double dSegLen = 0;
          int d = (*it).dim();
          for(int j = 0; j < d; j++)
          {
            dSegLen += ((*it)[j] - prePoint[j]) * ((*it)[j] - prePoint[j]);
          }
          dMovePathLen += sqrt(dSegLen); 
          prePoint = (*it);
        } 
      }

      ROS_INFO("nAttempts: %d", 20-nAttempts);
    }
    else        // 失败次数过多，需要补充roadmap节点
    {
      prm.Construct(500);
      nAttempts = 10;
    }

    gettimeofday(&timer_, NULL);
    dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
    dRemainingTime = dAllowedPlanningTime - (dCurrentTime - dStartTime);
    ROS_INFO("dRemainingTime: %f", dRemainingTime);
  }

  dMovePlanningTime = dCurrentTime - dStartTime;
  cout << "MovePlanningTime: " << dMovePlanningTime << endl;

  // 3.规划两端的上车运动，即Picking planning和Placing planning
  bool bPickSuccess, bPlaceSuccess;
  bPickSuccess = bPlaceSuccess = false;
  if(bMoveSuccess)    // 下车规划成功，则规划两端的上车
  {
    moveit::planning_interface::MoveGroupInterface upper_group("upper");
    upper_group.setPlannerId(upperPlannerName);
    // 设置起吊端起始、终止位形, pick_init_rs和pick_goal_rs在上面已求得
    upper_group.setStartState(pick_init_rs);        // 设置起始位形
    upper_group.setJointValueTarget(pick_goal_rs);  // 设置目标位形
    // 设置就位端起始、终止位形, place_init_rs和place_goal_rs在上面已求得
    upper_group.setStartState(place_init_rs);          // 设置起始位形
    upper_group.setJointValueTarget(place_goal_rs);    // 设置目标位形

    // 求解规划问题
    while(dRemainingTime > 0.0 && (!bPickSuccess || !bPlaceSuccess))
    {
      // 起吊端规划
      if(!bPickSuccess)
      {
        upper_group.setPlanningTime(dRemainingTime);
        bPickSuccess = upper_group.plan(pick_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(bPickSuccess)
        {
          ROS_INFO("PICKING SUCCESS!!");
        }
        else
        {
          ROS_INFO("FAILURE!!");
        }
      }
      gettimeofday(&timer_, NULL);
      dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
      dRemainingTime = dAllowedPlanningTime - (dCurrentTime - dStartTime);

      // 就位端规划
      if(!bPlaceSuccess)
      {
        upper_group.setPlanningTime(dRemainingTime);
        bPlaceSuccess = upper_group.plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(bPlaceSuccess)
        {
          ROS_INFO("PLACING SUCCESS!!");
        }
        else
        {
          ROS_INFO("FAILURE!!");
        }
      }
      gettimeofday(&timer_, NULL);
      dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
      dRemainingTime = dAllowedPlanningTime - (dCurrentTime - dStartTime);
      ROS_INFO("dRemainingTime: %f", dRemainingTime);
    }
  }

  gettimeofday(&timer_, NULL);
  dCurrentTime = timer_.tv_sec+(timer_.tv_usec/1000000.0);
  double dTotalPlanningTime = dCurrentTime - dStartTime;

  bool bFinalSuccess = bMoveSuccess && bPickSuccess && bPlaceSuccess;
  double dUpperJointWeights[4] = {1.0, 1.0, 1.0, 1.0};
  double dPathLen;
  if(bFinalSuccess)
    dPathLen = dMovePathLen + computePathLength(pick_plan, dUpperJointWeights) + computePathLength(place_plan, dUpperJointWeights);
  else
    dPathLen = 10e-40;
  std::string strFileName = resultPath_ + "/decompose/PRMBased+" + upperPlannerName + ".csv";
	std::ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
  static bool bFirstWrite = true;
	if( bFirstWrite )
	{
		fout << "找到路径" << ",路径长度" << ",总规划时间" << ",行走规划时间";
		bFirstWrite = false;
	}
	fout.seekp( 0, std::ios::end );  
	fout << "\n" << bFinalSuccess << "," << dPathLen << "," << dTotalPlanningTime << "," << dMovePlanningTime;
	fout.close();

  return bFinalSuccess;
}

bool MotionPlanningForCrane::generateValidStateByIK(const geometry_msgs::Pose& pose, planning_scene::PlanningScenePtr scene, robot_state::RobotState& rs)
{
  int nMaxSamples = 30000;
  bool bFound = false;
  int i = 0;
  //const robot_state::JointModelGroup* joint_model_group = scene->getCurrentState().getJointModelGroup("whole");
  while(i<nMaxSamples)
  {
    // 求逆解
    bool bIK = rs.setFromIK(joint_model_group_, pose, 2, 0.5);
    
    // 判断是否碰撞
    if(bIK && !scene->isStateColliding(rs, "whole"))
    {
       // 同步到rviz中，用于调试
       moveit_msgs::RobotState rs_msg;
       robot_state::robotStateToRobotStateMsg(rs, rs_msg_);
       jointPub_.publish(rs_msg_.joint_state);
       ROS_INFO("%f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
       rs.printStatePositions();
       sleep(2);

      bFound = true;
      break;
    }
    i++;
  }

  return bFound;
}

double MotionPlanningForCrane::computePathLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan, double* dJointWeights)
{
  double dLen = 0.0;
  trajectory_msgs::JointTrajectory joint_traj = plan.trajectory_.joint_trajectory;
  int nWaypoints = joint_traj.points.size();
  int nJoints = joint_traj.points[0].positions.size();
  //ROS_INFO("%d-------------------%d", nWaypoints, nJoints);
  
  double dSegLen;
  for(int i = 1; i < nWaypoints; i++)
  {
    dSegLen = 0.0;
    for(int j = 0; j < nJoints; j++)
    {
       dSegLen += dJointWeights[j] * pow((joint_traj.points[i].positions[j] - joint_traj.points[i-1].positions[j]), 2);
    }
    dLen += sqrt(dSegLen);
  }

  return dLen;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_planning_for_crane");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

//   rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
//   visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint","/AAAAAAAAAAAAA"));

// while(ros::ok())
// {
//   // Create pose
// Eigen::Affine3d pose, pose1;
// visual_tools_->generateRandomPose(pose);
// visual_tools_->generateRandomPose(pose1);
// // pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
// // pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

// // Publish arrow vector of pose
// ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
// visual_tools_->publishCuboid(visual_tools_->convertPoseToPoint(pose), visual_tools_->convertPoseToPoint(pose1), rviz_visual_tools::RED);
// sleep(1);
// }


//    MotionPlanningForCrane mpc;
//    std::vector<std::string> planners;
//   planners.push_back("RRTConnectkConfigDefault");
//   planners.push_back("SBLkConfigDefault");
//   planners.push_back("ESTkConfigDefault");
//   planners.push_back("LBKPIECEkConfigDefault");
//   planners.push_back("BKPIECEkConfigDefault");
//   planners.push_back("KPIECEkConfigDefault");
//   planners.push_back("RRTkConfigDefault");
//   planners.push_back("RRTstarkConfigDefault");
//   // //planners.push_back("TRRTkConfigDefault");   //似乎无法正常运行，move_group崩溃，需要设置额外的参数
//   planners.push_back("PRMkConfigDefault");
//   planners.push_back("PRMstarkConfigDefault");
////   planners.push_back("CBiMRRTConfigDefault");
//   mpc.samplingSingleConfigTest(planners, 50000);

    MotionPlanningForCrane mpc;
    mpc.prepareGenerateData("/home/lys/PycharmProjects/LiftSceneRepresentation/LiftSceneData/bin_map.txt");
    mpc.generateLocQualityMap("/home/lys/PycharmProjects/LiftSceneRepresentation/goal_loc_quality_map.txt", false);
    //mpc.generateValidConfs("kj");
    //mpc.planBySamplingSingleConfig("RRTstarkConfigDefault");

  // std::vector<std::string> planners;
  // planners.clear();
  // // planners.push_back("SBLkConfigDefault");
  // // planners.push_back("ESTkConfigDefault");
  // planners.push_back("LBKPIECEkConfigDefault");
  // planners.push_back("BKPIECEkConfigDefault");
  // planners.push_back("KPIECEkConfigDefault");
  // planners.push_back("RRTkConfigDefault");
  // planners.push_back("RRTConnectkConfigDefault");
  // planners.push_back("RRTstarkConfigDefault");
  // // //planners.push_back("TRRTkConfigDefault");   //似乎无法正常运行，move_group崩溃，需要设置额外的参数
  // planners.push_back("PRMkConfigDefault"); 
  // planners.push_back("PRMstarkConfigDefault");
  // mpc.decomposingTest(planners, planners, 100);

  // bool bSuccess = mpc.planBasedPRM("RRTkConfigDefault", 60);
//  bool bSuccess = mpc.planBySamplingSingleConfig("SBLkConfigDefault");
//   if( bSuccess)
//     ROS_INFO("SUCCESS");
//   else
//     ROS_INFO("FAILURE");

/*
  CraneLocationRegions pickCLR(mpc.initPose_);
  CraneLocationRegions placeCLR(mpc.goalPose_);
  Problem* pPro = new Problem("whole", &pickCLR, &placeCLR);
  BiMRRTs planner(pPro);


  bool bSuccess = planner.Plan();
  robot_trajectory::RobotTrajectoryPtr traj;
  traj.reset(new robot_trajectory::RobotTrajectory(mpc.whole_group_->getRobotModel(), "whole"));
  ConstructRobotTrajectory crt(mpc.whole_group_->getRobotModel(), "whole");
  crt.convertPath(planner.Path, *traj);
  if( bSuccess)
    ROS_INFO("SUCCESS");
  else
    ROS_INFO("FAILURE");
  
  moveit_msgs::RobotTrajectory trajectory;
  traj->getRobotTrajectoryMsg(trajectory) ;
  ros::Publisher display_publisher_;
  display_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory_;
  moveit_msgs::RobotState rs_msg;
  robot_state::robotStateToRobotStateMsg(traj->getFirstWayPoint(), rs_msg);
  display_trajectory_.trajectory_start = rs_msg;
  display_trajectory_.trajectory.push_back(trajectory);
  while(ros::ok())
  {
    display_publisher_.publish(display_trajectory_);
    sleep(20);
  }
*/
  ros::shutdown();  
  return 0;
}


