#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "crane_location_regions.h"

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "visialize_clr");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<geometry_msgs::Point> init_confs, goal_confs;
    std::ifstream fin("/home/lys/catkin_ws/src/crawler_crane/crane_tutorials/motion_planning_results/RRTConnectkConfigDefault.csv");
    if(fin)
    {
        std::string str;
        std::vector<std::string> data;
        geometry_msgs::Point pt;
        getline(fin, str);   // 去除第一行（标题）
        while(getline(fin, str))
        {
            //ROS_INFO("%s", str.c_str());
            split(str, ",", data);
            //ROS_INFO("(%f, %f)", atof(data[0].c_str()), atof(data[1].c_str()));
            bool bSuccess = atof(data[14].c_str());
            if(bSuccess)
            {
                pt.x = atof(data[0].c_str());
                pt.y = atof(data[1].c_str());
                pt.z = 0.0;
                init_confs.push_back(pt);

                pt.x = atof(data[7].c_str());
                pt.y = atof(data[8].c_str());
                pt.z = 0.0;
                goal_confs.push_back(pt);
            }
        }
    }
    else
    {
        ROS_INFO("Can't open the file!");
    }

    std::vector< CraneLocationRegions > init_clr_list, goal_clr_list;
    if( !CraneLocationRegions::getCLRsFromParamServer("init_clr_list", init_clr_list))
    {
        ROS_INFO("从参数服务器获取起吊站位扇环失败！！");
        return 0;
    }
    if( !CraneLocationRegions::getCLRsFromParamServer("goal_clr_list", goal_clr_list))
    {
        ROS_INFO("从参数服务器获取就位站位扇环失败！！");
        return 0;
    }

    geometry_msgs::Pose init_lift_obj_pose, goal_lift_obj_pose;
    init_lift_obj_pose.position.x = init_clr_list[0].liftObjPose_.dX;
    init_lift_obj_pose.position.y = init_clr_list[0].liftObjPose_.dY;
    init_lift_obj_pose.position.z = 0.05;
    goal_lift_obj_pose.position.x = goal_clr_list[0].liftObjPose_.dX;
    goal_lift_obj_pose.position.y = goal_clr_list[0].liftObjPose_.dY;
    goal_lift_obj_pose.position.z = 0.05;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, conf_visual_tools;
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom", "/moveit_visual_markers"));
    conf_visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("odom", "/conf_visual_markers"));
    visual_tools_->deleteAllMarkers();
    conf_visual_tools->deleteAllMarkers();

    std::cout << init_clr_list[0].annularSectors_[0].dMaxR << "   " << init_clr_list[0].annularSectors_[0].dMinR << std::endl;

    while(ros::ok()) {
        visual_tools_->publishCylinder(init_lift_obj_pose, rviz_visual_tools::colors::BLUE,
                0.2, 2*init_clr_list[0].annularSectors_[0].dMaxR, "Cylinder");

        visual_tools_->publishCylinder(init_lift_obj_pose, rviz_visual_tools::colors::RED,
                                       0.5, 2*init_clr_list[0].annularSectors_[0].dMinR, "Cylinder");
        visual_tools_->publishCylinder(goal_lift_obj_pose, rviz_visual_tools::colors::GREY,
                                       0.2, 2*goal_clr_list[0].annularSectors_[0].dMaxR, "Cylinder");
        visual_tools_->publishCylinder(goal_lift_obj_pose, rviz_visual_tools::colors::RED,
                                       0.5, 2*goal_clr_list[0].annularSectors_[0].dMinR, "Cylinder");
        visual_tools_->trigger();

        conf_visual_tools->publishSpheres(init_confs, rviz_visual_tools::colors::RED, 0.2, "Init_confs");
        conf_visual_tools->publishSpheres(goal_confs, rviz_visual_tools::colors::YELLOW, 0.2, "Goal_confs");
        conf_visual_tools->trigger();

        sleep(1);
    }
    return 0;
}