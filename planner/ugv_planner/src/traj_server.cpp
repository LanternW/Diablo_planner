
#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "ugv_planner/DifferentialVelocity.h"
#include "ugv_planner/traj_visualization.h"
#include "ugv_planner/Polynome.h"
#include "minco_opt/minco.hpp"

#include <minco_opt/poly_traj_utils.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

using namespace ugv_planner;
using namespace std;

ros::Publisher control_cmd_pub;
ros::Publisher despoint_vis_pub;
ros::Publisher despoint_pub;

const double pi = 3.1415926535;
double dt = 0.1;
double t_cur;

bool has_traj = false;
bool has_odom = false;

Trajectory trajectory;
minco::MinJerkOpt jerkOpt_;
double traj_duration;
ros::Time start_time;
int traj_id;


//odometry on real time
Eigen::Vector3d  odometry_pos;
double           odometry_yaw;


void tempRenderAPoint(Eigen::Vector3d pt, Eigen::Vector3d color)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id  = "world";
    sphere.header.stamp     = ros::Time::now();
    sphere.type             = visualization_msgs::Marker::SPHERE;
    sphere.action           = visualization_msgs::Marker::ADD;
    sphere.id               = 1;
    sphere.pose.orientation.w   = 1.0;
    sphere.color.r              = color(0);
    sphere.color.g              = color(1);
    sphere.color.b              = color(2);
    sphere.color.a              = 0.8;
    sphere.scale.x              = 0.2;
    sphere.scale.y              = 0.2;
    sphere.scale.z              = 0.2;
    sphere.pose.position.x      = pt(0);
    sphere.pose.position.y      = pt(1);
    sphere.pose.position.z      = pt(2);
    despoint_vis_pub.publish(sphere);
    
}





void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  if(has_odom == false){ cout <<"[TRAJ_SERVER] has odometry "<<endl; }
  has_odom = true;
  odometry_pos[0] = msg->pose.pose.position.x;
  odometry_pos[1] = msg->pose.pose.position.y;
  odometry_pos[2] = msg->pose.pose.position.z;
  Eigen::Quaterniond q( msg->pose.pose.orientation.w,
			                  msg->pose.pose.orientation.x,
		                  	msg->pose.pose.orientation.y,
		                  	msg->pose.pose.orientation.z );
  Eigen::Matrix3d R(q);
  odometry_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  
}


void polynomialTrajCallback(ugv_planner::PolynomeConstPtr msg)
{
  // parse pos traj
  Eigen::MatrixXd posP(3, msg -> pos_pts.size() - 2);
  Eigen::VectorXd T(msg -> t_pts.size());
  Eigen::MatrixXd initS, tailS;

  for (int i = 1; i < msg -> pos_pts.size() - 1 ;i++)
  {
    posP(0, i-1) = msg->pos_pts[i].x;
    posP(1, i-1) = msg->pos_pts[i].y;
    posP(2, i-1) = msg->pos_pts[i].z;
  }
  for (int i=0; i<msg->t_pts.size();i++)
  {
    T(i) = msg->t_pts[i];
  }
  
  initS.setZero(3, 3);
  tailS.setZero(3, 3);
  initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
  initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
  initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
  tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
  tailS.col(1) = Eigen::Vector3d::Zero();
  tailS.col(2) = Eigen::Vector3d::Zero();
  jerkOpt_.reset(initS, msg->pos_pts.size()-1);
  jerkOpt_.generate(posP, tailS, T);

  trajectory    = jerkOpt_.getTraj();
  traj_duration = trajectory.getTotalDuration();

  start_time  = msg -> start_time;
  traj_id     = msg -> traj_id;

  if(has_traj == false){ cout <<"[TRAJ_SERVER] has trajectory "<<endl; }
  has_traj = true;
}


void cmdCallback(const ros::TimerEvent &e)
{
    // no publishing before receive traj
    if ((!has_traj) || (!has_odom))
      return;
    
    ros::Time time_now  = ros::Time::now();
    t_cur               = (time_now - start_time).toSec();

    if (t_cur < traj_duration && t_cur >= 0.0)
    {
      Eigen::Vector3d des_pos   = trajectory.getPos(t_cur);
      tempRenderAPoint(des_pos, Eigen::Vector3d(0.1,0.2,0.9) );
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber traj_sub      = nh.subscribe("trajectory_topic", 10, polynomialTrajCallback);
  ros::Subscriber odom_sub      = nh.subscribe("odom", 1, rcvOdomCallBack );

  despoint_pub      = nh.advertise<geometry_msgs::PoseStamped>("despoint", 20); 
  despoint_vis_pub  = nh.advertise<visualization_msgs::Marker>("point/vis", 20); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);


  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}