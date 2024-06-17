//
// Created by lys on 2019/11/3.
//
#if __cplusplus <= 199711L
  #error This library needs at least a C++11 compliant compiler
#endif

#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <random_numbers/random_numbers.h>
random_numbers::RandomNumberGenerator rng;

#include "crane_location_regions.h"
#include <torch/script.h> // One-stop header.
#include <iomanip>

using namespace std;

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

bool isInterSection(double x_min, double x_max, double y_min, double y_max, double org_x, double org_y, double r, double R)
{
    double center_x = 0.5 * (x_min + x_max);
    double center_y = 0.5 * (y_min + y_max);
    double sqr_dist = (center_x - org_x) * (center_x - org_x) + (center_y - org_y) * (center_y - org_y);
    return  r * r < sqr_dist && sqr_dist < R * R;
}

void printVector(const vector<double>& v)
{
    unsigned int n = v.size();
    for(int i = 0; i < n; i++)
        cout << v[i];
    cout << endl;
}

void printArray(unsigned int** A, unsigned int m, unsigned int n)
{
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
            cout << A[i][j] << "  ";
        cout << endl;
    }
}

class CranePlanningDataGenerator {
public:
    CranePlanningDataGenerator();
    ~CranePlanningDataGenerator();

    bool loadGlobalMap(string global_map_file, unsigned int** global_map);
    bool loadLocalMap(string loc_map_file, unsigned int** loc_map);
    bool extractLocalMap(const geometry_msgs::Pose& pose, unsigned int **loc_map, bool bin);
    bool isCraneConfigValid(const std::vector<double>& craneConf, robot_state::RobotState& rs);
    bool generateValidStateByIK(const geometry_msgs::Pose& pose, planning_scene::PlanningScenePtr scene,
            robot_state::RobotState& rs);

    bool planBySamplingSingleConfig(string plannerName, robot_state::RobotState& init_rs,
            robot_state::RobotState& goal_rs);
    double computePathLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan, double* dJointWeights);
    int computeCellQualityForInitCLR(unsigned int i, unsigned int j, unsigned int N,
                                 const geometry_msgs::Pose& liftObj_pose_init,
                                 const geometry_msgs::Pose& liftObj_pose_goal,
                                 string plannerName);

    void generatePaths(const geometry_msgs::Pose& liftObj_pose_init, const geometry_msgs::Pose& liftObj_pose_goal,
            string plannerName);
    void generateValidConfs(const geometry_msgs::Pose& liftObj_pose_init, const geometry_msgs::Pose& liftObj_pose_goal,
                            bool for_init);
    int generateValidConfsForSingle(const geometry_msgs::Pose& liftObj_pose);
    void generateFreeConfs();
    vector<double> learningBasedSampler(const geometry_msgs::Pose& liftObj_pose);


    moveit::planning_interface::MoveGroupInterface* whole_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    planning_scene::PlanningScenePtr scene_;

    double local_map_size_;
    unsigned int local_map_n_;

    string data_path;

    // 深度学习获得的分布模型
    torch::jit::script::Module distribution_module;

    // 存储全局地图，从磁盘读取
    unsigned int global_map_n;
    unsigned int **global_bin_map;
    unsigned int **global_gray_map;
    // 存储当前局部地图，从磁盘读取
    unsigned int **init_local_map_;
    unsigned int **goal_local_map_;
    // 栅格化后起重机器人的潜在站位栅格
    unsigned int **init_potential_location_;
    unsigned int **goal_potential_location_;
    // 站位质量图，存储当前质量图，用以输出到磁盘
    unsigned int **init_loc_quality_map_;
    unsigned int **goal_loc_quality_map_;

    mutable random_numbers::RandomNumberGenerator rng_;

    // 统计时间
    timeval timer_;

    // 调试用
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    visualization_msgs::Marker markerVertexes_;
    void display(const geometry_msgs::Pose& liftObj_pose, unsigned int** loc_map);
    geometry_msgs::Point positionForIJ(const geometry_msgs::Pose& liftObj_pose, unsigned int i, unsigned int j);
};

geometry_msgs::Point CranePlanningDataGenerator::positionForIJ(const geometry_msgs::Pose& liftObj_pose,
        unsigned int i, unsigned int j)
{
    double x_min, y_min, step;
    step = local_map_size_ / local_map_n_;
    geometry_msgs::Point vertex;
    x_min = i * step -0.5 * local_map_size_ + liftObj_pose.position.x;
    y_min = j * step -0.5 * local_map_size_ + liftObj_pose.position.y;
    vertex.x = x_min + 0.5 * step;
    vertex.y = y_min + 0.5 * step;
    return vertex;
}

void CranePlanningDataGenerator::display(const geometry_msgs::Pose& liftObj_pose, unsigned int** loc_map)
{
    // 调试
    double x_min, y_min, step;
    step = local_map_size_ / local_map_n_;
    for(int i = 0; i < local_map_n_; i++)
    {
        for(int j = 0; j < local_map_n_; j++)
        {
            //printf("%d  ", this->init_local_map_[i][j]);
            if(loc_map[i][j])
            {
                geometry_msgs::Point vertex_;
                x_min = i * step -0.5 * local_map_size_ + liftObj_pose.position.x;
                y_min = j * step -0.5 * local_map_size_ + liftObj_pose.position.y;
                vertex_.x = x_min + 0.5 * step;
                vertex_.y = y_min + 0.5 * step;
                markerVertexes_.points.push_back(vertex_);
            }
        }
        //printf("\n");
    }
    int k = 0;
    while(k < 10000)
    {
        visual_tools_->publishMarker(markerVertexes_);
        visual_tools_->trigger();
        k++;
    }
}

CranePlanningDataGenerator::CranePlanningDataGenerator()
{
    data_path = std::string("/home/lys/PycharmProjects/LearnSampleDistribution/crane_planning_data/");
    int i, j;
    global_map_n = 250;
    global_bin_map = new unsigned int* [global_map_n];
    global_gray_map = new unsigned int* [global_map_n];
    for(i = 0; i < global_map_n; i++)
    {
        global_bin_map[i] = new unsigned int [global_map_n];
        global_gray_map[i] = new unsigned int [global_map_n];
    }
    // 从磁盘加载全局地图
    string global_bin_map_file = data_path + "global_bin_map.txt";
    string global_gray_map_file = data_path + "global_gray_map.txt";
    loadGlobalMap(global_bin_map_file, global_bin_map);
    loadGlobalMap(global_gray_map_file, global_gray_map);

    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        distribution_module = torch::jit::load("/home/lys/PycharmProjects/LearnSampleDistribution/th_models/init(复件)/traced_model.pt");
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }

    local_map_size_ = 56;
    local_map_n_ = 56;

    // 为相关数组分配内存空间
    init_local_map_ = new unsigned int* [local_map_n_];
    goal_local_map_ = new unsigned int* [local_map_n_];
    init_potential_location_ = new unsigned int* [local_map_n_];
    goal_potential_location_ = new unsigned int* [local_map_n_];
    init_loc_quality_map_ = new unsigned int* [local_map_n_];
    goal_loc_quality_map_ = new unsigned int* [local_map_n_];

    for(i = 0; i < local_map_n_; i++)
    {
        init_local_map_[i] = new unsigned int [local_map_n_];
        goal_local_map_[i] = new unsigned int [local_map_n_];
        init_potential_location_[i] = new unsigned int [local_map_n_];
        goal_potential_location_[i] = new unsigned int [local_map_n_];
        init_loc_quality_map_[i] = new unsigned int [local_map_n_];
        goal_loc_quality_map_[i] = new unsigned int [local_map_n_];
    }
    // 为相关数组赋初值--0
    for(i = 0; i < local_map_n_; i++)
    {
        for(j = 0; j < local_map_n_; j++)
        {
            init_local_map_[i][j] = 0;
            goal_local_map_[i][j] = 0;
            init_potential_location_[i][j] = 0;
            goal_potential_location_[i][j] = 0;
            init_loc_quality_map_[i][j] = 0;
            goal_loc_quality_map_[i][j] = 0;
        }
    }

    // 设置规划组及相关配置工作
    whole_group_ = new moveit::planning_interface::MoveGroupInterface("whole");
    robot_state::RobotState robot_state_ = robot_state::RobotState(*whole_group_->getCurrentState());
    joint_model_group_ = robot_state_.getJointModelGroup("whole");

    // 获得当前活动的场景，为碰撞检测做好准备
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef;
    monitor_ptr_udef.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
    ps->getCurrentStateNonConst().update();
    scene_ = ps->diff();
    scene_->decoupleParent();

    // 调试用
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
}

CranePlanningDataGenerator::~CranePlanningDataGenerator() {
    if(whole_group_)
        delete whole_group_;
}

bool CranePlanningDataGenerator::isCraneConfigValid(const std::vector<double>& craneConf, robot_state::RobotState& rs)
{
    rs.setJointGroupPositions(joint_model_group_, craneConf);
    scene_->setCurrentState(rs);
    robot_state::RobotState& current_state = scene_->getCurrentStateNonConst();
    bool bSatisfied = scene_->isStateValid(current_state, "whole");
    return bSatisfied;
}

bool CranePlanningDataGenerator::generateValidStateByIK(const geometry_msgs::Pose& pose, planning_scene::PlanningScenePtr scene, robot_state::RobotState& rs)
{
    int nMaxSamples = 3000;
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
//            // 同步到rviz中，用于调试
//            moveit_msgs::RobotState rs_msg;
//            robot_state::robotStateToRobotStateMsg(rs, rs_msg_);
//            jointPub_.publish(rs_msg_.joint_state);
//            ROS_INFO("%f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
//            rs.printStatePositions();
//            sleep(2);

            bFound = true;
            break;
        }
        i++;
    }

    return bFound;
}

bool CranePlanningDataGenerator::loadGlobalMap(string global_map_file, unsigned int** global_map)
{
    std::ifstream fin(global_map_file);
    int i = 0, j = 0;
    if(fin) {
        std::string str;
        std::vector<std::string> data;
        while (getline(fin, str)) {
            //ROS_INFO("%s", str.c_str());
            split(str, " ", data);
            for(j = 0; j < global_map_n; j++)
            {
                global_map[i][j] = atoi(data[j].c_str());
            }
            i++;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool CranePlanningDataGenerator::loadLocalMap(string loc_map_file, unsigned int** loc_map)
{
    std::ifstream fin(loc_map_file);
    int i = 0, j = 0;
    if(fin) {
        std::string str;
        std::vector<std::string> data;
        while (getline(fin, str)) {
            //ROS_INFO("%s", str.c_str());
            split(str, " ", data);
            for(j = 0; j < local_map_n_; j++)
            {
                loc_map[i][j] = atoi(data[j].c_str());
            }
            i++;
        }
    }
}

bool CranePlanningDataGenerator::extractLocalMap(const geometry_msgs::Pose& pose, unsigned int **loc_map, bool bin)
{
    int org_x_idx, org_y_idx, left_idx, bottom_idx;
    int i, j;
    org_x_idx = int(pose.position.x+125);
    org_y_idx = int(pose.position.y+125);
    left_idx = org_x_idx - int(0.5*local_map_n_);
    bottom_idx = org_y_idx - int(0.5*local_map_n_);
    unsigned int** global_map = bin ? global_bin_map : global_gray_map;
    for(i = 0; i < local_map_n_; i++)
    {
        for(j = 0; j < local_map_n_; j++)
        {
            if((0 <= left_idx+i && left_idx+i < global_map_n) && (0 <= bottom_idx+j && bottom_idx+j < global_map_n))  // 判断该单元格是否出了全局地图
            {
                loc_map[i][j] = global_map[left_idx + i][bottom_idx + j];
            }
            else
            {
                loc_map[i][j] = bin ? 1 : 256;
            }
        }
    }

    return true;
}

bool CranePlanningDataGenerator::planBySamplingSingleConfig(std::string plannerName, robot_state::RobotState& init_rs, robot_state::RobotState& goal_rs)
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
    double dJointWeights[7] = {1.0, 1.0, 10.0, 8.0, 5.0, 0.3, 0.1};
    double dPathLen;
    if(bPathFound)
        dPathLen = computePathLength(whole_plan, dJointWeights);
    else
        dPathLen = 10e-40;
    string resultPath = ros::package::getPath("crane_tutorials") + std::string("/crane_planning_data");
    string strFileName = resultPath + "/" + plannerName + ".csv";
    ofstream fout( strFileName.c_str(), std::ios::out|std::ios::app );
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

double CranePlanningDataGenerator::computePathLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan, double* dJointWeights)
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

int CranePlanningDataGenerator::computeCellQualityForInitCLR(unsigned int i, unsigned int j, unsigned int N,
                                                              const geometry_msgs::Pose& liftObj_pose_init,
                                                              const geometry_msgs::Pose& liftObj_pose_goal,
                                                              string plannerName)
{
    std::vector<double> base_conf(3, 0.0), crane_conf(8, 0.0);
    robot_state::RobotState init_rs(*whole_group_->getCurrentState()), goal_rs(*whole_group_->getCurrentState());
    CraneLocationRegions init_clr = CraneLocationRegions(liftObj_pose_init, 10.0, 28.0);
    double step_x, step_y;
    step_x = step_y = local_map_size_ / local_map_n_;
    unsigned n = 0;

    if(!init_potential_location_[i][j])
        return 0;     // 略过ij单元格

    for (int k = 0; k < N; k++) {
        base_conf[0] = rng_.uniformReal(i * step_x, (i + 1) * step_x);
        base_conf[1] = rng_.uniformReal(j * step_y, (j + 1) * step_y);
        base_conf[0] += -0.5 * local_map_size_ + liftObj_pose_init.position.x;
        base_conf[1] += -0.5 * local_map_size_ + liftObj_pose_init.position.y;
        base_conf[2] = rng_.uniformReal(-PI, PI);

        init_clr.upperIK(base_conf, crane_conf);
        if( isCraneConfigValid(crane_conf, init_rs) )
        {
            if( generateValidStateByIK(liftObj_pose_goal, scene_, goal_rs) )
            {
                if( planBySamplingSingleConfig(plannerName, init_rs, goal_rs) )
                {
                    n += 1;
                }
            }
        }
    }
    return n;
}

void CranePlanningDataGenerator::generatePaths(const geometry_msgs::Pose &liftObj_pose_init,
                                               const geometry_msgs::Pose &liftObj_pose_goal, string plannerName) {
    // 加载局部地图
    string dir = "/home/lys/catkin_ws/src/crawler_crane/crane_tutorials/crane_planning_data/";
    string init_local_map_file, goal_local_map_file;
    stringstream ss, ss1;
    ss << "(" << int(liftObj_pose_init.position.x) << "," << int(liftObj_pose_init.position.y)
       << "," << int(liftObj_pose_init.position.z) << ")_local_map.txt";
    init_local_map_file = ss.str();
    ss1 << "(" << int(liftObj_pose_goal.position.x) << "," << int(liftObj_pose_goal.position.y)
       << "," << int(liftObj_pose_goal.position.z) << ")_local_map.txt";
    goal_local_map_file = ss1.str();
    //loadLocalMap((dir + init_local_map_file).c_str(), this->init_local_map_);
    loadLocalMap((dir + goal_local_map_file).c_str(), this->goal_local_map_);

    // 计算起重机器人的潜在站位
    ROS_INFO("generatePaths");
    double x_min, x_max, y_min, y_max, step;
    step = local_map_size_ / local_map_n_;
    for(int i = 0; i < local_map_n_; i++)
    {
        for(int j = 0; j < local_map_n_; j++)
        {
            if(!this->init_local_map_[i][j])
            {
                x_min = i * step - 0.5 * local_map_size_ + liftObj_pose_init.position.x;
                x_max = x_min + step;
                y_min = j * step - 0.5 * local_map_size_ + liftObj_pose_init.position.y;
                y_max = y_min + step;
                this->init_potential_location_[i][j] = isInterSection(x_min, x_max, y_min, y_max,
                                                      liftObj_pose_init.position.x, liftObj_pose_init.position.y,
                                                      10.0, 28.0);
            }
        }
    }

    int i, j;
    int N = 2;

    // 求解起吊站位质量图
    for(i = 0; i < this->local_map_n_; i++)
    {
        for(j = 0; j < this->local_map_n_; j++)
        {
            // 计算单元格(i,j)的质量
            init_loc_quality_map_[i][j] = computeCellQualityForInitCLR(i, j, N, liftObj_pose_init, liftObj_pose_goal, plannerName);

            // 可视化该单元格
            if(init_loc_quality_map_[i][j] > 0)
            {
                geometry_msgs::Point pt = positionForIJ(liftObj_pose_init, i, j);
                markerVertexes_.points.push_back(pt);
                visual_tools_->publishMarker(markerVertexes_);
                visual_tools_->trigger();
            }

        }
    }

    // 存储起吊/就位CLR质量图
    string fileName = dir + "init_loc_quality_map.txt";
    ofstream fout(fileName);
    for(i = 0; i < local_map_n_; i++) {
        for (j = 0; j < local_map_n_; j++) {
            fout << init_loc_quality_map_[i][j] << " ";
        }
        fout << std::endl;
    }
}


void CranePlanningDataGenerator::generateValidConfs(const geometry_msgs::Pose& liftObj_pose_init,
        const geometry_msgs::Pose& liftObj_pose_goal, bool for_init){

    robot_state::RobotState rs(*whole_group_->getCurrentState());
    int nMaxSamples = 10000;
    int nValidSamples = 0;
    std::vector<geometry_msgs::Point> vis_points;
    geometry_msgs::Point pt;

    stringstream ss;
    ss << "(" << int(liftObj_pose_init.position.x) << "," << int(liftObj_pose_init.position.y)
       << "," << int(liftObj_pose_init.position.z) << ")_";
    ss << "(" << int(liftObj_pose_goal.position.x) << "," << int(liftObj_pose_goal.position.y)
        << "," << int(liftObj_pose_goal.position.z) << ")_conf_free";
    string file_name = ss.str();
    string data_path = ros::package::getPath("crane_tutorials") + std::string("/crane_planning_data/conf_free/");
    string init_file_name = data_path + file_name + "_init.txt";
    ofstream init_out( init_file_name.c_str() );
    string goal_file_name = data_path + file_name + "_goal.txt";
    ofstream goal_out( goal_file_name.c_str() );

    int j = 0;
    while (j < nMaxSamples) {
        //ROS_INFO("j: %d", j);
        // 求逆解
        bool bIK = rs.setFromIK(joint_model_group_, liftObj_pose_init, 2, 0.5);

        // 判断是否碰撞
        if (bIK && !scene_->isStateColliding(rs, "whole")) {
            nValidSamples += 1;
            double *joint_pos = rs.getVariablePositions();   // 0, 1, 2, 5, 6, 8, 9对应起重机位形
            // 输出无碰撞位形
            for(int k = 0; k < 10; k++)
            {
                if( k != 3 && k != 4 && k != 7 )
                    init_out << joint_pos[k] << ",";
            }
            init_out << endl;
            pt.x = joint_pos[0];
            pt.y = joint_pos[1];
            pt.z = 0.0;
            vis_points.push_back(pt);
            visual_tools_->publishSpheres(vis_points, rviz_visual_tools::colors::RED, 0.5, "confs");
            visual_tools_->trigger();
        }
        j++;
    }
    //sleep(1);

    j = 0;
    nValidSamples = 0;
    vis_points.clear();
    while (j < nMaxSamples) {
        //ROS_INFO("j: %d", j);
        // 求逆解
        bool bIK = rs.setFromIK(joint_model_group_, liftObj_pose_goal, 2, 0.5);

        // 判断是否碰撞
        if (bIK && !scene_->isStateColliding(rs, "whole")) {
            nValidSamples += 1;
            double *joint_pos = rs.getVariablePositions();   // 0, 1, 2, 5, 6, 8, 9对应起重机位形
            // 输出无碰撞位形
            for(int k = 0; k < 10; k++)
            {
                if( k != 3 && k != 4 && k != 7 )
                    goal_out << joint_pos[k] << ",";
            }
            goal_out << endl;
            pt.x = joint_pos[0];
            pt.y = joint_pos[1];
            pt.z = 0.0;
            vis_points.push_back(pt);
            visual_tools_->publishSpheres(vis_points, rviz_visual_tools::colors::GREEN, 0.5, "confs");
            visual_tools_->trigger();
        }
        j++;
    }
}

int CranePlanningDataGenerator::generateValidConfsForSingle(const geometry_msgs::Pose& liftObj_pose)
{
    robot_state::RobotState rs(*whole_group_->getCurrentState());
    int nMaxSamples = 10000;
    int nValidSamples = 0;
    std::vector<geometry_msgs::Point> vis_points;
    geometry_msgs::Point pt;

    stringstream ss;
    ss << "(" << int(liftObj_pose.position.x) << "," << int(liftObj_pose.position.y)
       << "," << int(liftObj_pose.position.z) << ")_confs_free.txt";
    string str = ss.str();
    string data_path = std::string("/home/lys/PycharmProjects/LearnSampleDistribution/crane_planning_data/local_confs/");
    string file_name = data_path + str;
    ofstream fout( file_name.c_str(), ios_base::app );

    int j = 0;
    while (j < nMaxSamples) {
        //ROS_INFO("j: %d", j);
        // 求逆解
        bool bIK = rs.setFromIK(joint_model_group_, liftObj_pose, 2, 0.5);

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
            fout << endl;
//            pt.x = joint_pos[0];
//            pt.y = joint_pos[1];
//            pt.z = 0.0;
//            vis_points.push_back(pt);
//            visual_tools_->publishSpheres(vis_points, rviz_visual_tools::colors::RED, 0.5, "confs");
//            visual_tools_->trigger();
        }
        j++;
    }

    return nValidSamples;
}

void CranePlanningDataGenerator::generateFreeConfs()
{
    const int N = 17;
//    double lift_obj_pos[N*3] = {-33.0, 15.0, 20.0, -38.0, -9.0, 25.0, -4.0, 45.0, 20.0, -4.0, 30.0, 5.0,
//                                 -4.0, -7.0, 5.0, -14.0, -47.0, 10.0, -60.0, 98.0, 25.0, -60.0, 80.0, 10.0,
//                                 -40.0, 70.0, 10.0, -45.0, 45.0, 10.0, -45.0, 14.0, 10.0, -30.0, -47.0, 10.0,
//                                 -30.0, -42.0, 10.0, -41.0, -16.0, 5.0, -18.0, 8.0, 12.0, 8.0, 110.0, 10.0,
//                                 -40.0, -50.0, 15.0};
//    geometry_msgs::Pose pose[N];
//    int i, j;
//
//    for(i = 0; i < N; i++)
//    {
//        pose[i].position.x = lift_obj_pos[i*3];
//        pose[i].position.y = lift_obj_pos[i*3+1];
//        pose[i].position.z = lift_obj_pos[i*3+2];
//        pose[i].orientation.w = 1;
//    }
//
//    CranePlanningDataGenerator gen;
//
//    for(i = 0; i < N; i++)
//    {
//        for(j = 0; j < N; j++)
//        {
//            //ROS_INFO("i=%d, j=%d", i, j);
//            if(i != j)
//                gen.generateValidConfs(pose[i], pose[j], true);
//        }
//    }


    // 构造被吊物位姿
    std::vector<geometry_msgs::Point> vis_points;
    geometry_msgs::Point pt;
    vector<geometry_msgs::Pose> poses;
    geometry_msgs::Pose pose;
    int x_base[3] = {-35, 0, 40};
    int ext = 25;
    int y_min = -100;
    int y_max = 110;
    int z_min = 1, z_max = 20;
    int num = 600;
    int i = 0;
    int j = 0;
    for(i = 0; i < 3; i++)
    {
        j = 0;
        while(j < num)
        {
            pose.position.x = rng.uniformInteger(x_base[i] - ext, x_base[i] + ext);
            pose.position.y = rng.uniformInteger(y_min, y_max);
            pose.position.z = rng.uniformInteger(z_min, z_max);
            pose.orientation.w = 1;
            poses.push_back(pose);

//            // 可视化
//            pt.x = pose.position.x;
//            pt.y = pose.position.y;
//            pt.z = 0; //pose.position.z;
//            vis_points.push_back(pt);
//            visual_tools_->publishSpheres(vis_points, rviz_visual_tools::colors::GREEN, 2, "confs");
//            visual_tools_->trigger();

            j++;
        }
//        ROS_INFO("Sleeping...");
//        sleep(3);
    }

    // 生成无碰撞位形
    int n = poses.size();
    string data_path = std::string("/home/lys/PycharmProjects/LearnSampleDistribution/crane_planning_data/");
    string file_name = data_path + "lift_obj_poses.txt";
    ofstream fout( file_name.c_str(), ios_base::app );
    int valid_confs_num = 0;
    for(i = 0; i < n; i++)
    {
        valid_confs_num = generateValidConfsForSingle(poses[i]);

        // 保存被吊物位姿
        if(valid_confs_num > 50)
        {
            fout << poses[i].position.x << "," << poses[i].position.y << "," << poses[i].position.z << endl;
        }
        ROS_INFO("Lift Object Pose [%d]!", i);
        //sleep(3);
    }
    fout.close();
}

vector<double> CranePlanningDataGenerator::learningBasedSampler(const geometry_msgs::Pose& liftObj_pose)
{
    extractLocalMap(liftObj_pose, init_local_map_, false);
//    printArray(global_gray_map, 250, 250);
//
//    float *p = new float[56*56];
//    for(int i = 0; i < 56; i++)
//    {
//        for(int j = 0; j < 56; j++)
//        {
//            p[56*i+j] = init_local_map_[i][j];
//        }
//    }
//
//    //display(liftObj_pose, init_local_map_);
//
//    torch::Tensor obs = torch::from_blob(p, {1, 56*56}, torch::kU8);
//    //cout << obs.to(torch::kU8) << endl;
//    for(int i = 0; i < 56; i++)
//    {
//        for(int j = 0; j < 56; j++)
//        {
//            cout << obs[56*i+j].item().toFloat() << "  ";
//        }
//        cout << endl;
//    }

    torch::Tensor obs = torch::zeros({1, 56*56});
    for(int i = 0; i < 56; i++)
    {
        for(int j = 0; j < 56; j++)
        {
            obs[0][56*i+j] = (int)init_local_map_[i][j];
        }
    }

//    for(int i = 0; i < 56; i++)
//    {
//        for(int j = 0; j < 56; j++)
//        {
//            cout << obs[0][56*i+j].item().toFloat() << "  ";
//        }
//        cout << endl;
//    }

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(obs);

    // Execute the model and turn its output into a tensor.
    at::Tensor output = distribution_module.forward(inputs).toTensor();

    int v;
    for(int i = 0; i < 56; i++)
    {
        for(int j = 0; j < 56; j++)
        {
            v = output[0][56*i+j].item().toInt();
            if(v >2)
            {
                init_loc_quality_map_[i][j] = v;
            }
        }
    }
    display(liftObj_pose, init_loc_quality_map_);

    vector<double> conf(8);
    return conf;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crane_planning_data_generator");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    geometry_msgs::Pose liftObj_pose_init, liftObj_pose_goal;
    liftObj_pose_goal.position.x = -33.0;
    liftObj_pose_goal.position.y = 15.0;
    liftObj_pose_goal.position.z = 20.0;
    liftObj_pose_goal.orientation.w = 1;
    liftObj_pose_init.position.x = -60.0;
    liftObj_pose_init.position.y = 80.0;
    liftObj_pose_init.position.z = 10.0;
    liftObj_pose_init.orientation.w = 1;

//    liftObj_pose_init.position.x = 8.0;
//    liftObj_pose_init.position.y = 110.0;
//    liftObj_pose_init.position.z = 10.0;
//    liftObj_pose_init.orientation.w = 1;
//    liftObj_pose_goal.position.x = -18.0;
//    liftObj_pose_goal.position.y = 8.0;
//    liftObj_pose_goal.position.z = 12;
//    liftObj_pose_goal.orientation.w = 1;

    //CranePlanningDataGenerator gen;
    //gen.generatePaths(liftObj_pose_init, liftObj_pose_goal, "RRTstarkConfigDefault");
    //gen.generateValidConfs(liftObj_pose_init, liftObj_pose_goal, true);

    CranePlanningDataGenerator gen;
    geometry_msgs::Pose lift_obj_pose;
    lift_obj_pose.position.x = 28.0;
    lift_obj_pose.position.y = -15.0;
    lift_obj_pose.position.z = 9.0;
    lift_obj_pose.orientation.w = 1;
    for(int i = 0; i < 10; i++)
    {
        lift_obj_pose.position.x = 2*i;
        lift_obj_pose.position.y = -85.0 + 10*i;
        gen.learningBasedSampler(lift_obj_pose);
        sleep(2);
    }


//    torch::jit::script::Module module;
//    try {
//        // Deserialize the ScriptModule from a file using torch::jit::load().
//        module = torch::jit::load("/home/lys/PycharmProjects/LearnSampleDistribution/th_models/init/traced_model.pt");
//    }
//    catch (const c10::Error& e) {
//        std::cerr << "error loading the model\n";
//        return -1;
//    }
//
//    std::cout << "ok\n";
//
//    std::vector<torch::jit::IValue> inputs;
//    inputs.push_back(torch::ones({1, 56*56}));
//
//    // Execute the model and turn its output into a tensor.
//    at::Tensor output = module.forward(inputs).toTensor();
//    std::cout << output << '\n';

    ros::shutdown();
    return 0;
}

