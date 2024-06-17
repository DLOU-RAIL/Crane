#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <iostream>
using namespace std;


int main(int argc, char **argv)
{
  ros::init (argc, argv, "test_octomap");
  ros::NodeHandle n;

  ros::Publisher octomap_pub = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  octomap::OcTree* octree = new octomap::OcTree("/home/hill/sample.bt");//命令行参数读入xxx.bt文件
  static octomap_msgs::Octomap octomap;
  octomap_msgs::binaryMapToMsg(*octree, octomap);//转换成消息格式
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.octomap.header.frame_id = "base_footprint";
  planning_scene.world.octomap.header.stamp = ros::Time::now();
  planning_scene.world.octomap.octomap.header.frame_id = "base_footprint";
  planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
  planning_scene.world.octomap.octomap.binary = true;
  planning_scene.world.octomap.octomap.id = "OcTree";
  planning_scene.world.octomap.octomap.resolution = 1;
  planning_scene.world.octomap.octomap.data = octomap.data;
  planning_scene.is_diff = true;
  

  while(ros::ok())
  {
    octomap_pub.publish(planning_scene);
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
    cout << octomap_pub.getNumSubscribers() << endl;;
  }
  
  while (octomap_pub.getNumSubscribers() < 1)
  {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
  }
  ROS_INFO("more than one subscriber, start publishing msgs on and on...");

}
