#pragma once
#include <ros/console.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class UgvManager{
public:
    void init(ros::NodeHandle &nh);
    void rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom);
    ros::Subscriber waypoints_sub, odom_sub;
    ros::Publisher visugv_pub;
private:
    std::string mesh_resource;
    std::string frame;
    double ugv_l, ugv_w, ugv_h;
};

//std::string mesh_resource;
//ros::Subscriber waypoints_sub;
//ros::Publisher viscar_pub;