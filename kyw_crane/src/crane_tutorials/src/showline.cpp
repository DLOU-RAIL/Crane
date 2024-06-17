/*
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "showline");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker line_list;
  ros::Rate r(60);
 tf::TransformListener listener;
  float f = 0.0;
  while (n.ok()){

    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    try{
      listener.waitForTransform("/base_footprint", "/rope_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/base_footprint", "/rope_link",  
                               ros::Time(0), transform1);
      listener.waitForTransform("/base_footprint", "/hook_block_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/base_footprint", "/hook_block_link",  
                               ros::Time(0), transform2);
    }

    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    line_list.header.frame_id = "/base_footprint";
    //line_list.header.stamp = ros::Time::now();
    //line_list.ns = "lines";
    
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 1;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    line_list.points.clear();
    geometry_msgs::Point p;
    p.x = transform1.getOrigin().x();
    p.y = transform1.getOrigin().y();
    p.z = transform1.getOrigin().z();
    // The line list needs two points for each line
ROS_ERROR("%f,%f,%f",p.x,p.y,p.z);
    line_list.points.push_back(p);
 
    p.x = transform2.getOrigin().x();
    p.y = transform2.getOrigin().y();
    p.z = transform2.getOrigin().z();
ROS_ERROR("%f,%f,%f",p.x,p.y,p.z);
    line_list.lifetime=ros::Duration(2);
    //transform2.liftime=ros::Duration();
    line_list.points.push_back(p);
    line_list.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(line_list);
    line_list.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(line_list);
    r.sleep();
  }
}
*/



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    //创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    //分配三个不同的id到三个markers。points_and_lines名称空间的使用确保彼此不会相互冲突。
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;


    //设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::TRIANGLE_LIST;



    // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 1;
    line_list.scale.y = 1;
    line_list.scale.y = 1;



    // 点为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip 是蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list 为红色
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // //使用正弦和余弦生成螺旋结构。POINTS和LINE_STRIP markers都仅仅需要1个点作为每个顶点，然而LINE_LIST marker需要2个点 。
    // for (uint32_t i = 0; i < 100; ++i)
    // {
    //   float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    //   float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = 0;
      p.y = 0;
      p.z = 0;

      // points.points.push_back(p);
      // line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      
      p.x = 2.0;
      p.y = 0.0;
      line_list.points.push_back(p);
      p.x = 0.0;
      p.y = 2.0;
      line_list.points.push_back(p);
    // }

    // //发布各个markers
    // marker_pub.publish(points);
    // marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}