
#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "ugv_planner/DifferentialVelocity.h"
#include "ugv_planner/traj_visualization.h"

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

#define L 0.5
#define OBJ_K 0.3
using namespace ugv_planner;
using namespace std;

ros::Publisher control_cmd_pub;
ros::Publisher despoint_vis_pub, despoint_vis_pub2;
ros::Publisher despoint_pub;

const double pi = 3.1415926535;
double dt = 0.1;


ugv_planner::DifferentialVelocity wheels_vel_cmd;

bool has_traj = false;
bool has_odom = false;


vector<UniformBspline> trajectory;
double traj_duration;
ros::Time start_time;
int traj_id;


//odometry on real time
Eigen::Vector3d  odometry_pos;
double           odometry_yaw;



//controller params
double t_cur;


double lfc = 0.3; //the desire point's distance

double time_forward_;
double last_yaw_, last_yaw_dot_;
double now_yaw, now_v, now_w;
double max_steer, v_max_, max_kappa;
bool last_nav_v = false;


void tempRenderAPoint(Eigen::Vector3d pt, Eigen::Vector3d color, bool use_2)
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
    if (use_2){despoint_vis_pub2.publish(sphere);}
    else{despoint_vis_pub.publish(sphere);}
    
}






















bool calcErrYaw(double &err_num, double des_yaw, double odom_yaw)
{
  bool dir = 0;  // 0 is clockwise, 1 is counterclockwise
  if(odom_yaw * des_yaw >= 0)  //odom and des are both in [0,pi] or [-pi, 0]
  {
    dir     = ((odom_yaw - des_yaw) >= 0) ? 0 : 1;
    err_num = fabs(odom_yaw - des_yaw);
  }
  else
  {
    double temp;
    bool   is_odom_positive;
    if(des_yaw < 0)      {temp = des_yaw  + 2 * pi; is_odom_positive = true; } 
    else if(odom_yaw < 0){temp = odom_yaw + 2 * pi; is_odom_positive = false;}

    if(is_odom_positive)
    {
      double dis1 = odom_yaw - des_yaw;
      double dis2 = temp - odom_yaw;
      if(dis1 < dis2){ dir = 0; err_num = dis1;}
      else           { dir = 1; err_num = dis2;}
    }
    else
    {
      double dis1 = des_yaw - odom_yaw;
      double dis2 = temp - des_yaw;
      if(dis1 < dis2){ dir = 1; err_num = dis1;}
      else           { dir = 0; err_num = dis2;}
    }

  }
  return dir;
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


void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time  = msg->start_time;
  traj_id     = msg->traj_id;

  trajectory.clear();
  trajectory.push_back(pos_traj);
  trajectory.push_back(trajectory[0].getDerivative());
  trajectory.push_back(trajectory[1].getDerivative());

  traj_duration = trajectory[0].getTimeSum();
  if(has_traj == false){ cout <<"[TRAJ_SERVER] has trajectory "<<endl; }
  has_traj = true;
}


void cmdCallback(const ros::TimerEvent &e)
{
    // no publishing before receive traj
    if ((!has_traj) || (!has_odom))
      return;

    ros::Time time_now  = ros::Time::now();
    t_cur               = (time_now - start_time).toSec() * OBJ_K;

    //{ cout <<"[TRAJ_SERVER] t_cur = "<< t_cur << "  traj_duration = " <<traj_duration<<endl; }

    //Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());

  double t_temp = t_cur;
  static double last_adv = 0.0;
  if (t_cur < traj_duration && t_cur >= 0.0)
  {
    while(t_temp < traj_duration && (odometry_pos - trajectory[0].evaluateDeBoorT(t_temp)).squaredNorm() < lfc)
    {
      t_temp += 0.1 * dt;
    }
    if(t_temp - t_cur > 0) {last_adv = t_temp - t_cur;}
    else                   {t_temp   = t_cur  + last_adv;}

    if (t_temp>=traj_duration)
    {
      t_temp = traj_duration;
      //ROS_ERROR("STOP SHITTING!!!");
      //ROS_INFO("vtemp = %f", trajectory[1].evaluateDeBoorT(t_temp).norm()) ; 
    }

    double desire_velocity    = trajectory[1].evaluateDeBoorT(t_temp).norm() * OBJ_K;            // ||desire velocity||
    Eigen::Vector3d des_pos   = trajectory[0].evaluateDeBoorT(t_temp);
    Eigen::Vector3d err_pos   = des_pos - odometry_pos;    //  desire position - odometry position
    
    //[DEBUG]
    //cout << "t = " << t_temp << " / " << traj_duration << " | v = " << trajectory[1].evaluateDeBoorT(t_temp).norm() * OBJ_K
    //     << " | a = " << trajectory[2].evaluateDeBoorT(t_temp).norm() * OBJ_K * OBJ_K <<endl;
    tempRenderAPoint(des_pos, Eigen::Vector3d(0.1,0.2,0.9), false);


    //[/DEBUG]

    double desire_yaw         = err_pos.norm() > 0.01 ? atan2(err_pos(1), err_pos(0)) : odometry_yaw;


    double err_yaw            = 0;
    bool err_dir              = calcErrYaw(err_yaw, desire_yaw, odometry_yaw);
    err_yaw                   = (err_dir == 0) ? -err_yaw : err_yaw;

    static double i_err_yaw = 0.0;
    static double d_err_yaw = 0.0;
    static double last_err_yaw = 0.0;
    i_err_yaw += err_yaw * dt;
    d_err_yaw  = (err_yaw - last_err_yaw)/dt;
    last_err_yaw = err_yaw;


    // controller input
    double c_velocity = desire_velocity + 0.0 * (err_pos.norm());
    double c_omega    = 5.1 * (err_yaw) + 0.4 * (i_err_yaw) + 3.5 * d_err_yaw;

    //{ cout <<"[TRAJ_SERVER] c_velocity = "<< c_velocity <<endl; }    
    //{ cout <<"[TRAJ_SERVER] c_omega = "<< c_omega <<endl; }

    wheels_vel_cmd.header.stamp = ros::Time::now();

    // dynamic model
    wheels_vel_cmd.left_wheel_vel  =  c_velocity - 0.5 * c_omega * L;
    wheels_vel_cmd.right_wheel_vel =  c_velocity + 0.5 * c_omega * L;

/*
    double tl = wheels_vel_cmd.right_wheel_vel * -1;
    double tr = wheels_vel_cmd.left_wheel_vel * -1;
    wheels_vel_cmd.left_wheel_vel = tl;
    wheels_vel_cmd.right_wheel_vel = tr;
*/

    //wheels_vel_cmd.left_wheel_vel  =  0;
    //wheels_vel_cmd.right_wheel_vel =  0;

    //wheels_vel_cmd.left_wheel_vel = wheels_vel_cmd.right_wheel_vel = 0.5;
    control_cmd_pub.publish(wheels_vel_cmd);
    geometry_msgs::PoseStamped desp;
    desp.header.stamp = ros::Time::now();
    desp.pose.position.x = des_pos(0);
    desp.pose.position.y = des_pos(1);
    desp.pose.position.z = des_pos(2);
    despoint_pub.publish(desp);

    des_pos   = trajectory[0].evaluateDeBoorT(t_cur);
    tempRenderAPoint(des_pos, Eigen::Vector3d(0.8,0.8,0.1), true);
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub   = nh.subscribe("trajectory_topic", 10, bsplineCallback);
  ros::Subscriber odom_sub      = nh.subscribe( "odom", 1, rcvOdomCallBack );

  control_cmd_pub   = nh.advertise<ugv_planner::DifferentialVelocity>("controller_cmd", 50);
  despoint_vis_pub  = nh.advertise<visualization_msgs::Marker>("point/vis", 20); 
  despoint_vis_pub2 = nh.advertise<visualization_msgs::Marker>("point/vis2", 20); 
  despoint_pub      = nh.advertise<geometry_msgs::PoseStamped>("despoint", 20); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);


  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}